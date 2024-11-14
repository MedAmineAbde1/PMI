#include <clocks.h>
#include <stm32l053xx.h>
#include <mcp23017.h>

#define I2C_MCP 0b00100000

int main(void)
{ 

    uint32_t result = 0;
    uint8_t buf = 0xf0;
    uint8_t dirbuff = 0x00;

    clocks_init_pmi();
    init_mcp23017();

    write_mcp23017(I2C_MCP, MCP_IODIRA, &dirbuff, 1);
    write_mcp23017(I2C_MCP, MCP_IODIRB, &dirbuff, 1);

    while (1)
    {
        result = write_mcp23017(I2C_MCP, MCP_GPIOA,  &buf, 1);
        result++;
        result = write_mcp23017(I2C_MCP, MCP_GPIOB,  &buf, 1);
        result++;
    }
    
}













