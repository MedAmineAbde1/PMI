#include <uart_irq.h>
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

#define MASTER //Setzt die Funktion des STM32 (MASTER/SLAVE)



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
    case 'y':
        buf = LED_Y;
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

/// @brief Zeichnet das Ampelsymbol auf dem Display
/// @param x Position x
/// @param y Position y
/// @param Ampelzeichen Buchstabe, welcher die Ampelphase kennzeichnet (analog zu set_led())
void draw_ampel(uint16_t x, uint16_t y, char Ampelzeichen)
{
    if (Ampelzeichen == 'g')
    {
        ili9341_draw_bmp_h(x, y, 49, 56, (uint8_t*) go_49x56, ILI9341_COLOR_GREEN, ILI9341_COLOR_BLACK);
        ili9341_draw_bmp_h(x, y+57, 49, 56, (uint8_t*) stop_51x56, ILI9341_COLOR_DARKGREY, ILI9341_COLOR_BLACK);
    }
    else if ((Ampelzeichen != 'O') && (Ampelzeichen != 'n'))
    {
        ili9341_draw_bmp_h(x, y, 49, 56, (uint8_t*) go_49x56, ILI9341_COLOR_DARKGREY, ILI9341_COLOR_BLACK);
        ili9341_draw_bmp_h(x, y+57, 49, 56, (uint8_t*) stop_51x56, ILI9341_COLOR_RED, ILI9341_COLOR_BLACK);
    }
    else
    {
        ili9341_draw_bmp_h(x, y, 49, 56, (uint8_t*) go_49x56, ILI9341_COLOR_DARKGREY, ILI9341_COLOR_BLACK);
        ili9341_draw_bmp_h(x, y+57, 49, 56, (uint8_t*) stop_51x56, ILI9341_COLOR_DARKGREY, ILI9341_COLOR_BLACK);
    }
}

/// @brief Schaltet die Mainstreet Ampel auf 'n' und die Sidestreet Ampel blinkt Gelb
void Blinking(void)
{
    set_led('M','n');
    set_led('S','y');
    systick_delay_ms(500);
    set_led('S','O');
    systick_delay_ms(500);
}



#ifdef SLAVE

char rx_buf[3] = ""; //Buffer zum Empfangen
char tx_buf[3] = "n\n"; //Buffer zum Senden
volatile uint8_t button = 0; //Status des Buttons/Induktionsschleife
uint8_t timeout = 0; //Status für Signaltimeout



int main(void)
{
    //Initialisierungsfunktionen
    clocks_init_pmi();
    ili9341_init(0);
    init_mcp23017();
    uart_irq_init_nucusb(115200);

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
    NVIC_SetPriority(EXTI0_1_IRQn, 3);
    NVIC_EnableIRQ(EXTI0_1_IRQn);

    //Interrupt Handler SW1
    EXTI->IMR |= EXTI_IMR_IM2;
    EXTI->FTSR |= EXTI_FTSR_FT2;
    NVIC_ClearPendingIRQ(EXTI2_3_IRQn);
    NVIC_SetPriority(EXTI2_3_IRQn, 3);
    NVIC_EnableIRQ(EXTI2_3_IRQn);

    //Interrupt Handler USART
    NVIC_ClearPendingIRQ(USART2_IRQn);
    NVIC_SetPriority(USART2_IRQn, 1);
    NVIC_EnableIRQ(USART2_IRQn);

    //Interrupt Handler TIM2
    RCC -> APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2 -> CR1 &= ~TIM_CR1_CEN; //Disable timer to be able to configure
    TIM2 -> PSC = 15999; //clock f=16 MHz |timer_f (16.000.000/PSC+1) and counts in range(0,ARR) in frequenz von timer_f  
    TIM2 -> ARR = 1500;
    TIM2 -> DIER  |= TIM_DIER_UIE;
    NVIC_ClearPendingIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn, 2);
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2 -> CR1 |= TIM_CR1_CEN; //Enable timer

    //Setzt die LED-Register auf Output
    uint8_t buf = 0x00;
    write_mcp23017(I2C_MCP, MCP_IODIRA, &buf, 1);
    write_mcp23017(I2C_MCP, MCP_IODIRB, &buf, 1);

    while (1)
    {
        //Abfrage, ob Timeout
        if (timeout)
        {
            Blinking();
        } else  {
            //Setzen der Ampel LEDs
            set_led('M', rx_buf[0]);
            set_led('S', rx_buf[1]);

            //Anzeige der Fußgängerampeln auf dem Display
            draw_ampel(20, 20, rx_buf[0]);
        }
    }
}

void EXTI0_1_IRQHandler ( void )
{
    if(EXTI->PR |= EXTI_PR_PIF1)
    {
        systick_delay_ms(200);
        button = 1;
        EXTI->PR &= ~EXTI_PR_PIF1;
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

/// @brief Interrupt Handler für TIM2
void TIM2_IRQHandler(void)
{
    if(TIM2 -> SR |= TIM_SR_UIF)
    {
        timeout = 1;
        TIM2 -> SR &= ~TIM_SR_UIF;
    }
}

/// @brief Interrupt Handler für Uart
void USART2_IRQHandler(void)
{
    if (USART2->ISR & USART_ISR_RXNE){
        //Schreibt die Eingabe von UART in rx_buf
        uint16_t len;
        uart_irq_rx_str(rx_buf, 3, &len);
        
        //Reset Counter
        TIM2->CNT = 0;

        //Reset Timeout
        timeout = 0;

        //Antwort an Master
        tx_buf[0] = (button == 1) ? 'y' : 'n';
        uart_irq_tx_str(tx_buf);
        button = 0;
    }
    if (USART2->ISR & USART_ISR_ORE) {
        USART2->ICR |= USART_ICR_ORECF;
    }
}
#endif



#ifdef MASTER

uint8_t timeout = 0; //Status für Signaltimeout
char rx_buf[3]; //Buffer zum Empfangen
char tx_buf[4] = "gg\n"; //Buffer zum Senden
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

/// @brief Arbeitet den aktuellen Zustand *state* ab
void stateMachine()
{
    //Setzen der LEDs
    set_led('M', state_table[state].led_character_ms);
    set_led('S', state_table[state].led_character_ss);

    //Zeichnet die Fußgängerampel auf das Display
    draw_ampel(20, 20, state_table[state].led_character_ms);

    //Setzt button zurück, wenn er in der falschen Ampelphase betätigt wurde
    if (state == SS_Y) button = 0;

    //Checkt, ob Taster gedrückt werden muss
    if (state_table[state].condition)
    {
        //Checkt, ob button gedrückt wurde
        while(button == 0);
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
    uart_irq_init_nucusb(115200);

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
    NVIC_SetPriority(EXTI0_1_IRQn, 3);
    NVIC_EnableIRQ(EXTI0_1_IRQn);

    //Interrupt Handler SW1
    EXTI->IMR |= EXTI_IMR_IM2;
    EXTI->FTSR |= EXTI_FTSR_FT2;
    NVIC_ClearPendingIRQ(EXTI2_3_IRQn);
    NVIC_SetPriority(EXTI2_3_IRQn, 3);
    NVIC_EnableIRQ(EXTI2_3_IRQn);

    //Interrupt Handler USART
    NVIC_ClearPendingIRQ(USART2_IRQn);
    NVIC_SetPriority(USART2_IRQn, 1);
    NVIC_EnableIRQ(USART2_IRQn);

    //Interrupt Handler Tim2
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2 -> CR1 &= ~TIM_CR1_CEN; // disable timer to be able to configure
    TIM2 -> PSC = 15999; //clock f=16 MHz |timer_f (16.000.000/PSC+1) and counts in range(0,ARR) in frequenz von timer_f
    TIM2 -> ARR = 1000;
    TIM2 -> DIER  |= TIM_DIER_UIE;
    NVIC_ClearPendingIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn, 2);
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2 -> CR1 |= TIM_CR1_CEN;

    //Setzt die LED-Register auf Output
    uint8_t buf = 0x00;
    write_mcp23017(I2C_MCP, MCP_IODIRA, &buf, 1);
    write_mcp23017(I2C_MCP, MCP_IODIRB, &buf, 1);

    while (1)
    {
        if (timeout)
        {
            Blinking();
        }
        else
        {
            stateMachine();
        }
    }
}

/// @brief Interrupt Handler für SW2
void EXTI0_1_IRQHandler ( void )
{
    if(EXTI->PR |= EXTI_PR_PIF1)
    {
        systick_delay_ms(200);
        button = 1;
        EXTI->PR &= ~EXTI_PR_PIF1;
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

/// @brief Interrupt Handler für TIM2
void TIM2_IRQHandler(void)
{
    if(TIM2 -> SR |= TIM_SR_UIF)
    {
        tx_buf[0] = state_table[state].led_character_ms;
        tx_buf[1] = state_table[state].led_character_ss;
        uart_irq_tx_str(tx_buf);
        timeout = 1;
        TIM2 -> SR &= ~TIM_SR_UIF;
    }    
}

/// @brief Interrupt Handler für Uart
void USART2_IRQHandler(void)
{
    if (USART2->ISR & USART_ISR_RXNE) {
        uint16_t len;
        uart_irq_rx_str(rx_buf, 3, &len);
        if (rx_buf[0] == 'y') button = 1;
        timeout = 0;
    }
    if (USART2->ISR & USART_ISR_ORE) {
        USART2->ICR |= USART_ICR_ORECF;
    }
}

#endif