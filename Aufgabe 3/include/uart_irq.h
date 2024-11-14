#ifndef uart_irq_H
#define uart_irq_H

#include <pmi_stddefs.h>

/**
 * @brief Initialises USART2 as uart for use with nucleo's serial-usb bridge
 * @param[in] baudrate Baud rate to be used
 * @return See @ref PMI_RC_E
 */
int32_t uart_irq_init_nucusb(uint32_t baudrate);

/**
 * @brief Transmits single character by polling
 * @param[in] c Charakter to transmit
 */
void uart_irq_tx_char(char c);

/**
 * @brief Receives a character by polling
 * @return Received character
 */
char uart_irq_rx_char(void);

void uart_irq_tx_str(char *buf);

int32_t uart_irq_rx_str(char *buf, uint16_t size, uint16_t *len);

#endif /* uart_irq_H */
