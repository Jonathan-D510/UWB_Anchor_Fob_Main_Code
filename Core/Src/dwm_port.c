#include "dwm_port.h"
#include "main.h"
#include "deca_device_api.h"
#include "deca_usleep.h"

void dw3000_port_init(void)
{
    /* Safe idle states */
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SPI_CLK_GPIO_Port, SPI_CLK_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SPI_MOSI_GPIO_Port, SPI_MOSI_Pin, GPIO_PIN_RESET);

    /* Module control */
    HAL_GPIO_WritePin(DWM_EXTON_GPIO_Port, DWM_EXTON_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DWM_WAKEUP_GPIO_Port, DWM_WAKEUP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DWM_RST_GPIO_Port, DWM_RST_Pin, GPIO_PIN_SET);

    deca_usleep(2000);
}

void dw3000_hard_reset(void)
{
    HAL_GPIO_WritePin(DWM_RST_GPIO_Port, DWM_RST_Pin, GPIO_PIN_RESET);
    deca_usleep(2000);
    HAL_GPIO_WritePin(DWM_RST_GPIO_Port, DWM_RST_Pin, GPIO_PIN_SET);
    deca_usleep(5000);
}

void dw3000_wakeup_pulse(void)
{
    HAL_GPIO_WritePin(DWM_WAKEUP_GPIO_Port, DWM_WAKEUP_Pin, GPIO_PIN_SET);
    deca_usleep(1000);
    HAL_GPIO_WritePin(DWM_WAKEUP_GPIO_Port, DWM_WAKEUP_Pin, GPIO_PIN_RESET);
    deca_usleep(1000);
}

void dw3000_on_exti(uint16_t gpio_pin)
{
#ifdef DWM_IRQ_Pin
    if (gpio_pin == DWM_IRQ_Pin)
    {
        dwt_isr();
    }
#else
    (void)gpio_pin;
#endif
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    dw3000_on_exti(GPIO_Pin);
}
