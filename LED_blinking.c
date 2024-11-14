#include "F28x_Project.h"
#define BLINKY_LED_GPIO1 31
#define BLINKY_LED_GPIO2 34

void main(void)
{

    InitSysCtrl();
    InitGpio();
    GPIO_SetupPinMux(BLINKY_LED_GPIO1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinMux(BLINKY_LED_GPIO2, GPIO_MUX_CPU1, 0);

    GPIO_SetupPinOptions(BLINKY_LED_GPIO1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinOptions(BLINKY_LED_GPIO2, GPIO_OUTPUT, GPIO_PUSHPULL);

    while(1)
    {
        GPIO_WritePin(BLINKY_LED_GPIO1, 0);
        GPIO_WritePin(BLINKY_LED_GPIO2, 0);

        DELAY_US(1000*500);
        // Turn off LED

        GPIO_WritePin(BLINKY_LED_GPIO1, 1);
        GPIO_WritePin(BLINKY_LED_GPIO2, 1);
        
        DELAY_US(1000*500);
    }
}
