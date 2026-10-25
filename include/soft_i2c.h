#ifndef SOFT_I2C_H
#define SOFT_I2C_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    void (*scl_float)(void);       // SCL -> Input (Hi-Z, pulled up)
    void (*scl_low)(void);         // SCL -> Output low
    bool (*scl_read)(void);        // Read SCL level
    void (*sda_float)(void);       // SDA -> Input (Hi-Z, pulled up)
    void (*sda_low)(void);         // SDA -> Output low
    bool (*sda_read)(void);        // Read SDA level
    void (*delay_half_period)(void);// Half-clock delay (e.g. 5 us for 100 kHz)
    uint32_t stretch_timeout_us;   // Clock stretch safety limit (e.g. 10000 us)
    uint32_t half_period_us;       // Half-clock delay length in us, used to count the stretch timeout
} soft_i2c_hal_t;

void soft_i2c_init(const soft_i2c_hal_t *hal);
void soft_i2c_bus_clear(const soft_i2c_hal_t *hal);
bool soft_i2c_write(const soft_i2c_hal_t *hal, uint8_t addr7, const uint8_t *data, uint16_t len);
bool soft_i2c_read(const soft_i2c_hal_t *hal, uint8_t addr7, uint8_t *data, uint16_t len);

#endif