#include <mcp23017.h>

void init_mcp23017()
{
    i2c_hw_init();
}

int32_t write_mcp23017(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t buf_size)
{
    return i2c_hw_tx_device(addr, reg, buf, buf_size);
}

int32_t read_mcp23017(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t buf_size)
{
    return i2c_hw_rx_device(addr, reg, buf, buf_size);
}