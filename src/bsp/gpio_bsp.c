#include <bsp/pwm_out_bsp.h>
#include <ch32v20x_gpio.h>

void gpio_bsp_init(void)
{
    GPIO_InitTypeDef gpio_init_strusture;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_PD01, ENABLE);

    GPIO_StructInit(&gpio_init_strusture);
    
    gpio_init_strusture.GPIO_Mode = GPIO_Mode_IPU;
    gpio_init_strusture.GPIO_Speed = GPIO_Speed_50MHz;

    /* For K-Online pins: PC13\PC14\PC15. */
    gpio_init_strusture.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOC, &gpio_init_strusture);
    /* For K-Online pins: PD0 */
    gpio_init_strusture.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOD, &gpio_init_strusture);

    /* For K-Pull pins: PB12\PB13\PB14\PB15. */
    gpio_init_strusture.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &gpio_init_strusture);
}

