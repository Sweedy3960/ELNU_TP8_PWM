/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdbool.h>

#include "17400.h"
#include "stm32delays.h"
#include "stm32driverlcd.h"
#include "time.h"
#include "stm32f0xx_it.h"
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
e_States state = INIT;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
bool flag1Ms ;//=false;
static bool InitialisationHasOccured = false;
// ----------------------------------------------------------------

// ADC 
// Lecture d'un canal AD par polling
// Il faut auparavant avoir configuré la/les pin(s) concernée(s) en entrée analogique
int16_t Adc_read(uint8_t chNr)
{	
	//démarage périph
	HAL_ADC_Start(&hadc);
	//calibration ADC 
	//HAL_ADCEx_Calibration_Start(&hadc);
	
	if(!HAL_ADC_PollForConversion(&hadc,5))
	{//timeout 5ms
		return HAL_ADC_GetValue(&hadc);
	}
	//a= (&hadc)->Instance->DR;
	HAL_ADC_Stop(&hadc);
	return 0;
}	

void LectureDuFlag1ms(void)
//cette fonction vérifie si le flag de 1ms est actif pour calculer les timmings demander 
//de plus la lecture des entré est faite lorsque que le flag est actif ce qui permet d'avoir 
//une base de temps pour les echantillons du buffer d'entrée au détriment du temps de réaction
{
	
	static uint16_t cntTime = 0;
	if(flag1Ms)
	{
		flag1Ms=false; 
		
		cntTime++;
		
		if (cntTime%_25MSEC == 0)
		{
			state = (InitialisationHasOccured)? EXEC: INIT;
			if(cntTime < _3SEC)
			{
				if(!(cntTime %_250MSEC))
				{
					GPIOC -> ODR ^= LEDS;
				}
			}
			else
			{
				InitialisationHasOccured=true;
				GPIOC -> ODR |= LEDS;
				cntTime = _3SEC;
				state = EXEC;
			}
		}		
	}
}

void initmotors(servo*servo,dcmot*dcmot)
{
	//constructeur ??
	dcmot->timerX=16;
	dcmot->speed_Percent=0;
	GPIOB->ODR |= MOT_EN;
	servo->timerX=17;
	
}
void setpulseregister(mototrs *motUsed)
{
	TIM16->CCR1= motUsed->motor1.speed_Percent;
	TIM17->CCR1= motUsed->servo1.pwmPercent_Tick;

}

void initialisation(mototrs *motUsed)
//Affichage durant l'initialisation partie LCD 
{
	if (InitialisationHasOccured)
	{
	}
	else
	{
		
		printf_lcd("TP AdLcd <2024>");
		lcd_gotoxy(1,2);
		printf_lcd("ACL EDA  ");
		lcd_bl_on();
		//constructeur ??
		initmotors(&motUsed->servo1,&motUsed->motor1);
		setpulseregister(motUsed);
	}
}

void valueAdcToSpeedDir(int16_t* adcVal,dcmot *moteur1)
{
	 int16_t adval = *adcVal;
	 int16_t coef = -49;
	 moteur1->speed_Percent = abs((((adval*coef)/1000)+100));
	 //
	 moteur1->sens = (adval <=2040)? '>':(moteur1->speed_Percent !=0)?'<':'-';
	
	switch (moteur1->sens)
	{
		case '>':
			GPIOB->ODR &= ~MOT_DIR;
			break;
		case '<':
			GPIOB->ODR |= MOT_DIR;
			break;
		default:
			GPIOB->ODR |= MOT_DIR;
			break;
	}
	
	 
}

void angleToTicks(int *consigneAngle,servo *moteur2)
{
	int coef = 96;
	int consignAngle = *consigneAngle;
	moteur2->pwmPercent_Tick = (abs(((consignAngle)*coef)+(_06MSTOTICK)));
	moteur2->angleDegree = *consigneAngle;
	moteur2->sens = (consignAngle <90)? '-':(consignAngle >90)?'+':' ';
}
void readInput(char *tb_portEntree)
// cette fonction remplis toute les 5ms une case du tableau avec le port d'entrée
//une fois plein le tableau est annalyser, la taille du tableau est faite pour les limites cdc 500ms
{
	
		tb_portEntree[2] = tb_portEntree[1];
		tb_portEntree[1] = tb_portEntree[0];
		tb_portEntree[0] = GPIOC-> IDR & 0x0F;
	
	
}
void inputsActions(char *tb_portEntree,int *servoConsigne)
{
	static bool modefin = false;
	int consignAngle = *servoConsigne;
	//Test pour pulse temps actif min 200ms
			if ((tb_portEntree[0] != tb_portEntree[1])&& (tb_portEntree[0]!=0))
			{
				switch (((~(tb_portEntree[0]))&0xF))
				{
						case S2 :
							modefin = !modefin;
							break;
						
						case S3:
							if (consignAngle >=1)
							{
								if (modefin)
								{
									consignAngle-=1;
								}
								else
								{
									consignAngle=(((consignAngle-10)/10)*10);
								}
							}
							break;
						case S4:
							if (consignAngle <180)
							{
								if (modefin)
								{
									consignAngle+=1;
								}
								else
								{
									consignAngle=(((consignAngle+10)/10)*10);
								}
							}
							break;
						case S5:
							
							consignAngle = 90;
							
							break;
				}		
				if(consignAngle > 180 || consignAngle < 0  )
				{
					
				}
				else
				{
					*servoConsigne = consignAngle;
				}
			}

}


void execution(mototrs *motUsed)
{
		static int16_t alors =0;
		
		static int servoConsigne = 0;
		static char tb_portEntree[3]={0};
	
		alors=Adc_read(0);
		valueAdcToSpeedDir(&alors,&motUsed->motor1);
		readInput(tb_portEntree);
		inputsActions(tb_portEntree,&servoConsigne);
		angleToTicks(&servoConsigne,&motUsed->servo1);
		setpulseregister(motUsed);
		lcd_gotoxy(1,1);
		printf_lcd("SPEED: %d / % DIR: %c ",motUsed->motor1.speed_Percent, motUsed->motor1.sens);
		lcd_gotoxy(1,2);
		printf_lcd("ANGLE: %c %d          ",(motUsed->servo1.sens),abs(motUsed->servo1.angleDegree-90));
}
// ----------------------------------------------------------------

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//variables
	// *** A COMPLETER ! ***

	
	//init	
	// *** A COMPLETER ! ***
	

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM6_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_ADC_Init();
	lcd_init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim16);
	HAL_TIM_Base_Start_IT(&htim17);
	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_ADCEx_Calibration_Start(&hadc);
	static mototrs motUsed;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		LectureDuFlag1ms();
		
		/*
		char test;
		test = Adc_read(1);	
		TIM16->CCR1=1;
		TIM17->CCR1=1;
		*/
		switch(state)
		{
			case INIT:
				
				state = IDLE;
				initialisation(&motUsed);
				break;
			case EXEC:
				
				state = IDLE;
				execution(&motUsed);
				break;
			case IDLE:
				
				break;
			default: 
				break;
		}
		
		// *** A COMPLETER ! ***
		
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
