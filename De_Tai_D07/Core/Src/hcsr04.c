#include "hcsr04.h"
#include "tim.h"

#define SHOT_PERIOD_MS 70U
#define ECHO_TIMEOUT_MS 35U
#define IDLE 0U
#define WAIT_RISE 1U
#define WAIT_FALL 2U
static volatile uint8_t state, ready;
static volatile uint16_t rise;
static volatile uint32_t shot_tick;
static uint32_t last_shot;
static volatile HCSR04_Result result;

static uint32_t Lock(void)
{
    uint32_t previous = __get_PRIMASK();
    __disable_irq();
    return previous;
}
static void Unlock(uint32_t previous) { __set_PRIMASK(previous); }

/* Goi trong ISR hoac khi da khoa ngat. */
static void Finish(HCSR04_Status status, uint32_t width)
{
    __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC1);
    state = IDLE;
    result.echo_us = width;
    result.status = status;
    ready = 1U;
}

static HAL_StatusTypeDef Trigger(void)
{
    /* PWM2: CNT=0 LOW; CNT=1..12 HIGH, sau update ve LOW.
       OPM tu dung counter sau mot chu ky. */
    if (HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2) != HAL_OK) return HAL_ERROR;
    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    htim2.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
    return HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}

HAL_StatusTypeDef HCSR04_Init(void)
{
    state = IDLE;
    ready = 0U;
    rise = 0U;
    last_shot = HAL_GetTick();
    shot_tick = last_shot;
    htim2.Instance->CR1 |= TIM_CR1_OPM;
    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    htim2.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
    if (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1) != HAL_OK) return HAL_ERROR;
    __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC1);
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_CC1 | TIM_FLAG_CC1OF);
    return HAL_OK;
}

void HCSR04_Task(void)
{
    uint32_t key, now;
    key = Lock();
    now = HAL_GetTick();
    if (state != IDLE && (uint32_t)(now - shot_tick) >= ECHO_TIMEOUT_MS)
        Finish(HCSR04_TIMEOUT, 0U);
    if (state == IDLE && !ready && (uint32_t)(now - last_shot) >= SHOT_PERIOD_MS) {
        last_shot = now;
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET) {
            Finish(HCSR04_ECHO_HIGH, 0U);
        } else {
            __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
            __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_CC1 | TIM_FLAG_CC1OF);
            shot_tick = now;
            state = WAIT_RISE;
            __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC1);
            if (Trigger() != HAL_OK) Finish(HCSR04_HW_ERROR, 0U);
        }
    }
    Unlock(key);
}

void HCSR04_CaptureCallback(TIM_HandleTypeDef *htim)
{
    uint16_t captured, width;
    if (htim->Instance != TIM3 || htim->Channel != HAL_TIM_ACTIVE_CHANNEL_1) return;
    if (state == IDLE) return;
    if ((uint32_t)(HAL_GetTick() - shot_tick) >= ECHO_TIMEOUT_MS) {
        Finish(HCSR04_TIMEOUT, 0U);
        return;
    }
    if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC1OF) != RESET) {
        __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC1OF);
        Finish(HCSR04_BAD_PULSE, 0U);
        return;
    }
    captured = (uint16_t)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    if (state == WAIT_RISE) {
        rise = captured;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        state = WAIT_FALL;
    } else {
        /* Tru unsigned 16 bit xu ly ca truong hop counter tran. */
        width = (uint16_t)(captured - rise);
        Finish(width >= 100U && width <= 25000U ? HCSR04_OK : HCSR04_BAD_PULSE, width);
    }
}

uint8_t HCSR04_GetResult(HCSR04_Result *destination)
{
    uint32_t key;
    uint8_t available;
    if (destination == 0) return 0U;
    key = Lock();
    available = ready;
    if (available) {
        destination->echo_us = result.echo_us;
        destination->status = result.status;
        ready = 0U;
    }
    Unlock(key);
    return available;
}
