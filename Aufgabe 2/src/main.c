#include <clocks.h>
#include <stm32l053xx.h>
#include <mcp23017.h>
#include <systick.h>
#include <ili9341.h>
#include <bmp_go_49x56.h>
#include <bmp_stop_51x56.h>



//Registerwerte für die entsprechenden LEDs
#define LED_O ~0b00000000
#define LED_R ~0b00000001
#define LED_Y ~0b00000010
#define LED_G ~0b00000100
#define LED_r ~0b00001000
#define LED_g ~0b00010000

#define I2C_MCP 0b0100000 //I2C Adresse von MCP



volatile uint8_t button = 0; //Variable für den Status des Buttons
typedef enum {MS_G1=0, MS_Y, MS_R, SS_D, SS_G, SS_Y, SS_R, MS_D, MS_G2, MS_G3} state_t; //Zustandsliste aller Ampelzustände
state_t state = MS_G1; //Aktueller Zustand

//Aumsetzung der Zustandstabelle
typedef struct main
{
    char led_character_ms;
    char led_character_ss;
    uint8_t condition;
    uint8_t wait_time;
    state_t next_state;
} tl_state_t ;

tl_state_t state_table[] = 
{
    {'G', 'g', 1, 3,  MS_Y},
    {'Y', 'g', 0, 1,  MS_R},
    {'R', 'R', 0, 2,  SS_D},
    {'R', 'D', 0, 1,  SS_G},
    {'g', 'G', 0, 10, SS_Y},
    {'R', 'Y', 0, 1,  SS_R},
    {'R', 'R', 0, 1,  MS_D},
    {'D', 'R', 0, 1,  MS_G2},
    {'G', 'R', 0, 1,  MS_G3},
    {'G', 'g', 0, 3,  MS_G1}
};



/// @brief Setzt die LEDs (led) der Straße (Street)
/// @param street 'M': Mainstreet, 'S': Sidestreet
/// @param led 'O': Alle aus; 'R': alle Rot; 'g': Verkehr rot, Fußgänger grün; 'Y': Verkehr gelb, Fußgänger rot; 'r': Verkehr aus, Fußgänger rot; 'G': Verkehr grün, Fußgänger rot; 'n': Verkehr grün, Fußgänger aus; 'D': Verkehr rot+gelb, Fußgänger rot
/// @return 0: Gültige Eingabe, -1: Fehler
uint8_t set_led(char street, char led)
{
    //Setzt das zu beschreibende Register (Main street: MCP_GPIOA, Side street: MCP_GPIOB)
    uint8_t reg; 
    switch (street)
    {
    case 'M':
        reg = MCP_GPIOA;
        break;
    case 'S':
        reg = MCP_GPIOB;
        break;
    default:
        return -1;
    }

    //Schaltet alle LEDs im Register reg ab
    uint8_t buf = LED_O;
    write_mcp23017(I2C_MCP, reg, &buf, 1);

    //Switch Case für die einzuschaltenden LEDs
    switch (led)
    {
    case 'O':
        return 0;
    case 'R':
        buf = LED_R & LED_r;
        write_mcp23017(I2C_MCP, reg, &buf, 1);
        return 0;
    case 'g':
        buf = LED_R & LED_g;
        write_mcp23017(I2C_MCP, reg, &buf, 1);
        return 0;
    case 'Y':
        buf = LED_Y & LED_r;
        write_mcp23017(I2C_MCP, reg, &buf, 1);
        return 0;
    case 'r':
        buf = LED_r;
        write_mcp23017(I2C_MCP, reg, &buf, 1);
        return 0;
    case 'G':
        buf = LED_G & LED_r;
        write_mcp23017(I2C_MCP, reg, &buf, 1);
        return 0;
    case 'n':
        buf = LED_G;
        write_mcp23017(I2C_MCP, reg, &buf, 1);
        return 0;
    case 'D':
        buf = LED_R & LED_Y & LED_r;
        write_mcp23017(I2C_MCP, reg, &buf, 1);
        return 0;
    default:
        return -1;
    }
}



/// @brief Arbeitet den aktuellen Zustand *state* ab
void stateMachine()
{
    //Setzen der LEDs
    set_led('M', state_table[state].led_character_ms);
    set_led('S', state_table[state].led_character_ss);

    //Anzeige der MS Fußgängerampel auf dem Display
    if (state_table[state].led_character_ms == 'g')
    {
        ili9341_draw_bmp_h(20, 20, 49, 56, (uint8_t*) go_49x56, ILI9341_COLOR_GREEN, ILI9341_COLOR_BLACK);
        ili9341_draw_bmp_h(20, 77, 49, 56, (uint8_t*) stop_51x56, ILI9341_COLOR_DARKGREY, ILI9341_COLOR_BLACK);
    }
    else if ((state_table[state].led_character_ms != 'O') || (state_table[state].led_character_ms != 'n'))
    {
        ili9341_draw_bmp_h(20, 20, 49, 56, (uint8_t*) go_49x56, ILI9341_COLOR_DARKGREY, ILI9341_COLOR_BLACK);
        ili9341_draw_bmp_h(20, 77, 49, 56, (uint8_t*) stop_51x56, ILI9341_COLOR_RED, ILI9341_COLOR_BLACK);
    }

    //Anzeige der SS Fußgängerampel auf dem Display
    if (state_table[state].led_character_ss == 'g')
    {
        ili9341_draw_bmp_h(77, 20, 49, 56, (uint8_t*) go_49x56, ILI9341_COLOR_GREEN, ILI9341_COLOR_BLACK);
        ili9341_draw_bmp_h(77, 77, 49, 56, (uint8_t*) stop_51x56, ILI9341_COLOR_DARKGREY, ILI9341_COLOR_BLACK);
    }
    else if ((state_table[state].led_character_ss != 'O') || (state_table[state].led_character_ms != 'n'))
    {
        ili9341_draw_bmp_h(77, 20, 49, 56, (uint8_t*) go_49x56, ILI9341_COLOR_DARKGREY, ILI9341_COLOR_BLACK);
        ili9341_draw_bmp_h(77, 77, 49, 56, (uint8_t*) stop_51x56, ILI9341_COLOR_RED, ILI9341_COLOR_BLACK);
    }

    //Setzt button zurück, wenn er in der falschen Ampelphase betätigt wurde
    if (state == SS_Y) button = 0;

    //Checkt, ob Taster gedrückt werden muss
    if (state_table[state].condition)
    {
        //Checkt, ob button gedrückt wurde
        while(1)
        {
            if (button == 1) break;
        }
        button = 0;
    }

    //Wartezeit
    systick_delay_ms(state_table[state].wait_time * 1000);

    //Wechsel in den nächsten Zustand
    state = state_table[state].next_state;
}



int main(void)
{
    //Initialisierungsfunktionen
    clocks_init_pmi();
    ili9341_init(0);
    init_mcp23017();

    RCC->IOPENR |= RCC_IOPENR_GPIOBEN; //Enable the Register the port B
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; //Enable the Peripheral clock register

    SYSCFG -> EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PB;  
    SYSCFG -> EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PB;

    //setzt GPIOB auf Inputs Modus an.
    GPIOB->MODER &= ~(GPIO_MODER_MODE1);
    GPIOB->MODER &= ~(GPIO_MODER_MODE2);

    //Interrupt Handler SW2
    EXTI->IMR |= EXTI_IMR_IM1;
    EXTI->FTSR |= EXTI_FTSR_FT1;
    NVIC_ClearPendingIRQ(EXTI0_1_IRQn);
    NVIC_SetPriority(EXTI0_1_IRQn, 2);
    NVIC_EnableIRQ(EXTI0_1_IRQn);

    //Interrupt Handler SW1
    EXTI->IMR |= EXTI_IMR_IM2;
    EXTI->FTSR |= EXTI_FTSR_FT2;
    NVIC_ClearPendingIRQ(EXTI2_3_IRQn);
    NVIC_SetPriority(EXTI2_3_IRQn, 2);
    NVIC_EnableIRQ(EXTI2_3_IRQn);

    //Setzt die LED-Register auf Output
    uint8_t buf = 0x00;
    write_mcp23017(I2C_MCP, MCP_IODIRA, &buf, 1);
    write_mcp23017(I2C_MCP, MCP_IODIRB, &buf, 1);

    while (1)
    {
        stateMachine();
    }
}


/// @brief Interrupt Handler für SW2
void EXTI0_1_IRQHandler ( void )
{
    if(EXTI-> PR |= EXTI_PR_PIF1)
    {
        systick_delay_ms(200);
        button = 1;
        EXTI-> PR &= ~EXTI_PR_PIF1;
    }    
}


/// @brief Interrupt Handler für SW1
void EXTI2_3_IRQHandler ( void )
{
    if(EXTI->PR |= EXTI_PR_PIF2)
    {
        systick_delay_ms(200);
        button = 1;
        EXTI->PR &= ~EXTI_PR_PIF2;
    }
}