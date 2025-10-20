#include "stm32f4xx_hal.h"
#include "velocity_profile.h"
#include "ir_sensors.h"

#include <math.h>

ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim2;

static MotionConstraints g_constraints;

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void DriveMotors(float velocity);
static uint16_t ReadIRSensor(uint32_t channel);
static void RunSimpleTest(IRSensorArray *sensors);
static void Error_Handler(void);

static const uint32_t kSensorChannels[IR_SENSOR_COUNT] = {
    ADC_CHANNEL_9,
    ADC_CHANNEL_10,
    ADC_CHANNEL_14,
    ADC_CHANNEL_12,
    ADC_CHANNEL_15,
    ADC_CHANNEL_11};

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_ADC1_Init();

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    g_constraints.max_velocity = 0.8f;
    g_constraints.max_acceleration = 2.0f;
    g_constraints.max_deceleration = 2.0f;

    VelocityProfile profile;
    VelocityProfile_Init(&profile, g_constraints);
    VelocityProfile_Plan(&profile, 0.5f);

    IRSensorArray sensors;
    IRSensorArray_Init(&sensors, 5U, 0U);

    RunSimpleTest(&sensors);

    uint32_t start_tick = HAL_GetTick();

    while (1)
    {
        uint32_t now = HAL_GetTick();
        float elapsed = (now - start_tick) / 1000.0f;

        float position = 0.0f;
        float velocity = 0.0f;
        VelocityProfile_Sample(&profile, elapsed, &position, &velocity);
        DriveMotors(velocity);

        for (uint8_t i = 0U; i < IR_SENSOR_COUNT; ++i)
        {
            uint16_t sample = ReadIRSensor(kSensorChannels[i]);
            IRSensorArray_Update(&sensors, (IRSensorId)i, sample);
        }

        if (VelocityProfile_IsComplete(&profile, elapsed))
        {
            start_tick = now;
            VelocityProfile_Plan(&profile, 0.5f);
        }

        HAL_Delay(10);
    }
}

static void RunSimpleTest(IRSensorArray *sensors)
{
    if (sensors == NULL)
    {
        return;
    }

    for (uint16_t cycle = 0U; cycle < 50U; ++cycle)
    {
        DriveMotors(0.3f);

        for (uint8_t i = 0U; i < IR_SENSOR_COUNT; ++i)
        {
            uint16_t sample = ReadIRSensor(kSensorChannels[i]);
            IRSensorArray_Update(sensors, (IRSensorId)i, sample);
        }

        HAL_Delay(20U);
    }

    DriveMotors(0.0f);
}

static void DriveMotors(float velocity)
{
    float norm = 0.0f;
    if (g_constraints.max_velocity > 0.0f)
    {
        norm = velocity / g_constraints.max_velocity;
    }

    if (norm > 1.0f)
    {
        norm = 1.0f;
    }
    else if (norm < -1.0f)
    {
        norm = -1.0f;
    }

    uint32_t reload = __HAL_TIM_GET_AUTORELOAD(&htim2);
    uint32_t pulse = (uint32_t)(fabsf(norm) * (float)(reload));

    if (norm >= 0.0f)
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0U);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0U);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0U);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulse);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0U);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pulse);
    }
}

static uint16_t ReadIRSensor(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    sConfig.Offset = 0;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        return 0U;
    }

    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
        return 0U;
    }

    if (HAL_ADC_PollForConversion(&hadc1, 10U) != HAL_OK)
    {
        HAL_ADC_Stop(&hadc1);
        return 0U;
    }

    uint16_t value = (uint16_t)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return value;
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 84U - 1U;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 1000U - 1U;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0U;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_ADC1_Init(void)
{
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.NbrOfDiscConversion = 0;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *tim_pwmHandle)
{
    if (tim_pwmHandle->Instance == TIM2)
    {
        __HAL_RCC_TIM2_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;

        GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_11;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *tim_pwmHandle)
{
    if (tim_pwmHandle->Instance == TIM2)
    {
        __HAL_RCC_TIM2_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0 | GPIO_PIN_2);
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3 | GPIO_PIN_11);
    }
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *adcHandle)
{
    if (adcHandle->Instance == ADC1)
    {
        __HAL_RCC_ADC1_CLK_ENABLE();

        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;

        GPIO_InitStruct.Pin = GPIO_PIN_1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *adcHandle)
{
    if (adcHandle->Instance == ADC1)
    {
        __HAL_RCC_ADC1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_1);
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5);
    }
}

void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
}

static void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
    Error_Handler();
}
#endif
