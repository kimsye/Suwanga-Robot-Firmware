    /* USER CODE BEGIN Header */
    /**
     ******************************************************************************
    * @file           : main.c
    * @brief          : Main program body
    ******************************************************************************
    * @attention
    *
    * Copyright (c) 2026 STMicroelectronics.
    * All rights reserved.
    *
    * This software is licensed under terms that can be found in the LICENSE file
    * in the root directory of this software component.
    * If no LICENSE file comes with this software, it is provided AS-IS.
    *
    ******************************************************************************
    */
    /* USER CODE END Header */
    /* Includes ------------------------------------------------------------------*/
    #include "main.h"
    #include "adc.h"
    #include "usart.h"
    #include "gpio.h"

    /* Private includes ----------------------------------------------------------*/
    /* USER CODE BEGIN Includes */

    #include <stdio.h>
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
    uint32_t mux_adc[16] = {0};
    uint32_t adc_ind[4] = {0};
    uint8_t sw0 = 0;
    uint8_t sw1 = 0;
    char msg[256];

    uint8_t sw0_toggle = 0;
    uint8_t sw1_toggle = 0;

    uint8_t sw0_prev = 1;
    uint8_t sw1_prev = 1;

    /* USER CODE END PV */

    /* Private function prototypes -----------------------------------------------*/
    void SystemClock_Config(void);
    /* USER CODE BEGIN PFP */

    /* USER CODE END PFP */

    /* Private user code ---------------------------------------------------------*/
    /* USER CODE BEGIN 0 */
    void MUX_Select(uint8_t ch)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, (ch >> 0) & 0x01);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, (ch >> 1) & 0x01);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, (ch >> 2) & 0x01);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, (ch >> 3) & 0x01);
    }

    void Read_MUX_ADC(uint8_t mux_ch, uint32_t *value)
    {
        ADC_ChannelConfTypeDef sConfig = {0};

        // 1. MUX Channel selection
        MUX_Select(mux_ch);

        // 2. time
        for (volatile int i = 0; i < 300; i++);

        // 3. ADC == PA0 fix
        sConfig.Channel = ADC_CHANNEL_0;
        sConfig.Rank = 1;
        sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
        sConfig.Offset = 0;

        HAL_ADC_ConfigChannel(&hadc1, &sConfig);

        // 4. ADC transmitter
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        *value = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
    }

    void Read_ADC_Channel(uint32_t channel, uint32_t *value)
    {
        ADC_ChannelConfTypeDef sConfig = {0};

        sConfig.Channel = channel;
        sConfig.Rank = 1;
        sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
        sConfig.Offset = 0;

        HAL_ADC_ConfigChannel(&hadc1, &sConfig);

        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        *value = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
    }
    /* USER CODE END 0 */

    /**
     * @brief  The application entry point.
     * @retval int
     */
    int main(void)
    {

    /* USER CODE BEGIN 1 */

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
    MX_ADC1_Init();
    MX_USART2_UART_Init();
    /* USER CODE BEGIN 2 */
    HAL_GPIO_WritePin(GPIOB,
        GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6,
        GPIO_PIN_RESET);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        for (uint8_t i = 0; i < 16; i++)
                {
                    Read_MUX_ADC(i, &mux_adc[i]);
                }
        Read_ADC_Channel(ADC_CHANNEL_9, &adc_ind[0]);   // PB1
        Read_ADC_Channel(ADC_CHANNEL_10, &adc_ind[1]);  // PC0
        Read_ADC_Channel(ADC_CHANNEL_11, &adc_ind[2]);   // PC1
        Read_ADC_Channel(ADC_CHANNEL_4, &adc_ind[3]); // PA4



        sw0 = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_SET);
        sw1 = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET);

        /* PB2 */
        if (sw0_prev == 1 && sw0 == 0)
        {
            sw0_toggle ^= 1;
        }
        sw0_prev = sw0;

        /* PB10 */
        if (sw1_prev == 1 && sw1 == 0)
        {
            sw1_toggle ^= 1;
        }
        sw1_prev = sw1;



        int len = sprintf(msg,
        "%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%u,%u\r\n",

        mux_adc[0], mux_adc[1], mux_adc[2], mux_adc[3],
        mux_adc[4], mux_adc[5], mux_adc[6], mux_adc[7],
        mux_adc[8], mux_adc[9], mux_adc[10], mux_adc[11],
        mux_adc[12], mux_adc[13], mux_adc[14], mux_adc[15],

        adc_ind[0], adc_ind[1], adc_ind[2], adc_ind[3],

        sw0_toggle, sw1_toggle);

        HAL_UART_Transmit(
            &huart2,
            (uint8_t*)msg,
            len,
            HAL_MAX_DELAY
        );

        HAL_Delay(10);
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

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
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
    __disable_irq();
    while (1)
    {
    }
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
        ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
    }
    #endif /* USE_FULL_ASSERT */
