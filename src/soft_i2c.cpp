#include "soft_i2c.h"

static bool wait_scl_high(const soft_i2c_hal_t *hal) {
    hal->scl_float();
    uint32_t elapsed = 0;
    while (!hal->scl_read()) {
        hal->delay_half_period();
        elapsed += 5; // assumes nominal 5us half-period
        if (elapsed > hal->stretch_timeout_us) {
            return false; // Slave hung holding SCL low
        }
    }
    return true;
}

static void i2c_start(const soft_i2c_hal_t *hal) {
    hal->sda_float();
    hal->scl_float();
    hal->delay_half_period();
    hal->sda_low();
    hal->delay_half_period();
    hal->scl_low();
}

static void i2c_stop(const soft_i2c_hal_t *hal) {
    hal->sda_low();
    hal->delay_half_period();
    if (!wait_scl_high(hal)) {
        hal->scl_low(); // timed out, force clear
    }
    hal->delay_half_period();
    hal->sda_float();
    hal->delay_half_period();
}

static bool i2c_write_byte(const soft_i2c_hal_t *hal, uint8_t byte) {
    for (uint8_t i = 0; i < 8; i++) {
        if (byte & 0x80) {
            hal->sda_float();
        } else {
            hal->sda_low();
        }
        byte <<= 1;
        hal->delay_half_period();
        
        if (!wait_scl_high(hal)) return false;
        hal->delay_half_period();
        hal->scl_low();
    }

    // Read ACK/NACK bit
    hal->sda_float();
    hal->delay_half_period();
    if (!wait_scl_high(hal)) return false;
    
    bool ack = !hal->sda_read(); // ACK is active LOW
    hal->delay_half_period();
    hal->scl_low();
    return ack;
}

static uint8_t i2c_read_byte(const soft_i2c_hal_t *hal, bool ack) {
    uint8_t byte = 0;
    hal->sda_float();

    for (uint8_t i = 0; i < 8; i++) {
        byte <<= 1;
        hal->delay_half_period();
        if (!wait_scl_high(hal)) return 0xFF;
        if (hal->sda_read()) {
            byte |= 0x01;
        }
        hal->delay_half_period();
        hal->scl_low();
    }

    // Send ACK (low) or NACK (high/float)
    if (ack) {
        hal->sda_low();
    } else {
        hal->sda_float();
    }
    hal->delay_half_period();
    wait_scl_high(hal);
    hal->delay_half_period();
    hal->scl_low();
    hal->sda_float();

    return byte;
}

// 9-clock bus un-wedging routine
void soft_i2c_bus_clear(const soft_i2c_hal_t *hal) {
    hal->sda_float();
    for (int i = 0; i < 9; i++) {
        hal->scl_float();
        hal->delay_half_period();
        hal->scl_low();
        hal->delay_half_period();
        if (hal->sda_read()) break; // Released early
    }
    i2c_stop(hal);
}

bool soft_i2c_write(const soft_i2c_hal_t *hal, uint8_t addr7, const uint8_t *data, uint16_t len) {
    i2c_start(hal);
    if (!i2c_write_byte(hal, (addr7 << 1) | 0x00)) {
        i2c_stop(hal);
        return false;
    }
    for (uint16_t i = 0; i < len; i++) {
        if (!i2c_write_byte(hal, data[i])) {
            i2c_stop(hal);
            return false;
        }
    }
    i2c_stop(hal);
    return true;
}

bool soft_i2c_read(const soft_i2c_hal_t *hal, uint8_t addr7, uint8_t *data, uint16_t len) {
    i2c_start(hal);
    if (!i2c_write_byte(hal, (addr7 << 1) | 0x01)) {
        i2c_stop(hal);
        return false;
    }
    for (uint16_t i = 0; i < len; i++) {
        data[i] = i2c_read_byte(hal, (i < (len - 1))); // ACK all except the last byte
    }
    i2c_stop(hal);
    return true;
}