#include <clocks.h>
#include <stm32l053xx.h>
#include <mcp23017.h>
#include <systick.h>



#define I2C_MCP 0b0100000 //I2C Adresse von MCP

//Registerwerte für die entsprechenden LEDs
#define LED_O ~0b00000000
#define LED_R ~0b00000001
#define LED_Y ~0b00000010
#define LED_G ~0b00000100
#define LED_r ~0b00001000
#define LED_g ~0b00010000



uint8_t buf[] = {LED_G, LED_Y, LED_R, LED_R, LED_r, LED_g, LED_Y, LED_r, LED_g};
uint8_t richtung = 0; //Bestimmt die Drehrichtung der LEDs. 1: Im Uhrzeigersinn. 0: Gegen den Uhrzeigersinn.
uint8_t stop = 0; //Bestimmt ob die Lauflicht angehaltet wird oder nicht



void setLed(uint16_t data)
{
    uint8_t buf = data & 0x0f;
    write_mcp23017(I2C_MCP, MCP_GPIOA, &buf, 1);
    buf = (data >> 8) & 0x0f;
    write_mcp23017(I2C_MCP, MCP_GPIOB, &buf, 1);
}



int main(void)
{
    uint8_t dirbuff = 0x00;
    uint8_t bruh = LED_O;
    int i = 0; //Zählvariable für den aktuellen Zustand

    //Initialisierungsfunktionen
    clocks_init_pmi();
    init_mcp23017();

    //Setzt die IO Register auf Output
    write_mcp23017(I2C_MCP, MCP_IODIRA, &dirbuff, 1);
    write_mcp23017(I2C_MCP, MCP_IODIRB, &dirbuff, 1);

    RCC->IOPENR |= RCC_IOPENR_GPIOBEN; //Enable the Register|the port B
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; //Enable the Peripheral clock register

    //
    SYSCFG -> EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PB;  
    SYSCFG -> EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PB;

    //setzt GPIOB auf Inputs Modus an.
    GPIOB->MODER &= ~(GPIO_MODER_MODE1);
    GPIOB->MODER &= ~(GPIO_MODER_MODE2);

    //
    EXTI->IMR |= EXTI_IMR_IM1;
    EXTI->FTSR |= EXTI_FTSR_FT1;
    NVIC_ClearPendingIRQ(EXTI0_1_IRQn);
    NVIC_SetPriority(EXTI0_1_IRQn, 2);
    NVIC_EnableIRQ(EXTI0_1_IRQn);

    //
    EXTI->IMR |= EXTI_IMR_IM2;
    EXTI->FTSR |= EXTI_FTSR_FT2;
    NVIC_ClearPendingIRQ(EXTI2_3_IRQn);
    NVIC_SetPriority(EXTI2_3_IRQn, 2);
    NVIC_EnableIRQ(EXTI2_3_IRQn);
    
    while (1)
    {
        if (stop == 0)
        {
            if ((i >= 0 && i <= 2) || (i >= 7 && i <= 8))
            {
                write_mcp23017(I2C_MCP, MCP_GPIOA, &buf[i], 1);
                write_mcp23017(I2C_MCP, MCP_GPIOB, &bruh, 1);
            }
            else
            {
                write_mcp23017(I2C_MCP, MCP_GPIOA, &bruh, 1);
                write_mcp23017(I2C_MCP, MCP_GPIOB, &buf[i], 1);
            }
            richtung ? i >= 8 ? i = 0 : i++ : i <= 0 ? i = 8 : i--; //Abfrage, ob i hoch bzw. heruntergezählt wird.
        }
        systick_delay_ms(50);
    }   
}



/// @brief SW2 Interrupt Handler
void EXTI0_1_IRQHandler ( void )
{
    systick_delay_ms(200);
    if(EXTI->PR |= EXTI_PR_PIF1)
    {
        if (stop == 1) {
            stop = 0;
        } else {
            stop = 1;
        }
        EXTI-> PR &= ~EXTI_PR_PIF1;
    }    
}



/// @brief SW1 Interrupt Handler
void EXTI2_3_IRQHandler ( void )
{
    systick_delay_ms(200);
    if(EXTI->PR |= EXTI_PR_PIF2)
    {
        if (richtung == 1) {
            richtung = 0;
        } else {
            richtung = 1;
        }
        EXTI->PR &= ~EXTI_PR_PIF2;
    }    
}