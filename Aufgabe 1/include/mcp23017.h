#include <i2c.h>

#ifndef __MCP23017_H__
#define __MCP23017_H__


#define MCP_IODIRA 0x00
#define MCP_IODIRB 0x01
#define MCP_GPIOA  0x12
#define MCP_GPIOB  0x13

/**
 * @brief Initialises hardware i2c
 * @return See @ref PMI_RC_E
 */
void init_mcp23017();

int32_t write_mcp23017(uint8_t addr, uint8_t reg, uint8_t *buf,uint8_t buf_size);

int32_t read_mcp23017(uint8_t addr, uint8_t reg, uint8_t *buf,uint8_t buf_size);

#endif // MCP23017_H
