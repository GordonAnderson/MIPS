/**
 * @file  WireServer.cpp
 * @brief Implementation of WireServer — see WireServer.h for full docs.
 */

#include "WireServer.h"

// -------------------------------------------------------------------------- //
// Internal helpers                                                            //
// -------------------------------------------------------------------------- //

/**
 * Request @p count bytes from @p addr and verify that they all arrived.
 * Returns true only when Wire reports at least @p count bytes available.
 *
 * Note: Wire::requestFrom() is blocking; it returns after the transaction
 * completes or times out.  Checking available() afterward tells us whether
 * the peripheral ACKed and sent the expected number of bytes.
 */
static bool requestBytes(TwoWire *wire, uint8_t addr, uint8_t count)
{
    wire->requestFrom(addr, count);
    return (wire->available() >= count);
}

// -------------------------------------------------------------------------- //
// Construction / configuration                                                //
// -------------------------------------------------------------------------- //

WireServer::WireServer()
    : wire(&Wire), addr(0)
{
}

void WireServer::setInterface(TwoWire *twi)
{
    wire = twi;
}

// -------------------------------------------------------------------------- //
// Command transmission                                                        //
// -------------------------------------------------------------------------- //

bool WireServer::sendCommand(uint8_t address, const uint8_t *buffer,
                             uint8_t num, bool defer)
{
    // Store the target address for subsequent Read* calls.
    addr = address;

    wire->beginTransmission(addr);

    for (uint8_t i = 0; i < num; i++)
    {
        wire->write(buffer[i]);
    }

    // When deferred, leave the transmission open so the caller can append a
    // typed payload via one of the Send* helpers, which will close it.
    if (defer)
    {
        return true;
    }

    return (wire->endTransmission() == 0);
}

bool WireServer::endTransaction()
{
    return (wire->endTransmission() == 0);
}

// -------------------------------------------------------------------------- //
// Typed read helpers                                                          //
// -------------------------------------------------------------------------- //

bool WireServer::readInt32(int32_t *value)
{
    // Request all 4 bytes up front and verify they arrived before reading.
    // Using int32_t (not int) avoids undefined behaviour from shifting beyond
    // the width of a plain int on 16-bit AVR targets where int is 16 bits.
    if (!requestBytes(wire, addr, 4)) return false;

    // Reconstruct little-endian 32-bit value one byte at a time.
    // Casting each byte to uint32_t before shifting prevents sign-extension
    // from corrupting the upper bits when the byte value has bit 7 set.
    uint32_t raw;
    raw  = (uint32_t)wire->read();
    raw |= (uint32_t)wire->read() << 8;
    raw |= (uint32_t)wire->read() << 16;
    raw |= (uint32_t)wire->read() << 24;

    // Memcpy-style reinterpretation avoids strict-aliasing UB.
    memcpy(value, &raw, sizeof(*value));
    return true;
}

bool WireServer::readInt16(int16_t *value)
{
    if (!requestBytes(wire, addr, 2)) return false;

    // Read directly into the byte representation of the output variable.
    // Casting to uint8_t* is defined behaviour for any object type.
    uint8_t *b = reinterpret_cast<uint8_t *>(value);
    b[0] = wire->read(); // LSB
    b[1] = wire->read(); // MSB
    return true;
}

bool WireServer::readUint8(uint8_t *value)
{
    if (!requestBytes(wire, addr, 1)) return false;

    *value = wire->read();
    return true;
}

bool WireServer::readInt8(int8_t *value)
{
    if (!requestBytes(wire, addr, 1)) return false;

    *value = static_cast<int8_t>(wire->read());
    return true;
}

bool WireServer::readBool(bool *value)
{
    if (!requestBytes(wire, addr, 1)) return false;

    *value = (wire->read() != 0);
    return true;
}

bool WireServer::readFloat(float *value)
{
    if (!requestBytes(wire, addr, 4)) return false;

    // Read into the raw byte representation of the float.
    // IEEE 754 layout is assumed to match the sender (both are little-endian
    // AVR/ARM targets in the expected deployment environment).
    uint8_t *b = reinterpret_cast<uint8_t *>(value);
    for (uint8_t j = 0; j < 4; j++)
    {
        b[j] = wire->read();
    }
    return true;
}

// -------------------------------------------------------------------------- //
// Typed send helpers                                                          //
// -------------------------------------------------------------------------- //
// Each helper appends bytes to the transmission opened by sendCommand() with
// defer=true, then calls endTransmission() to close it.
// -------------------------------------------------------------------------- //

void WireServer::sendBool(bool value)
{
    wire->write(static_cast<uint8_t>(value ? 1 : 0));
    wire->endTransmission();
}

void WireServer::sendUint8(uint8_t value)
{
    wire->write(value);
    wire->endTransmission();
}

void WireServer::sendUint16(uint16_t value)
{
    // Send little-endian: LSB first.
    const uint8_t *b = reinterpret_cast<const uint8_t *>(&value);
    wire->write(b[0]); // LSB
    wire->write(b[1]); // MSB
    wire->endTransmission();
}

void WireServer::sendInt24(int32_t value)
{
    // Send the 3 least-significant bytes (little-endian).
    // The caller is responsible for ensuring the value fits in 24 bits.
    const uint8_t *b = reinterpret_cast<const uint8_t *>(&value);
    wire->write(b[0]);
    wire->write(b[1]);
    wire->write(b[2]);
    wire->endTransmission();
}

void WireServer::sendFloat(float value)
{
    // Reinterpret the float as 4 raw bytes and send little-endian.
    const uint8_t *b = reinterpret_cast<const uint8_t *>(&value);
    wire->write(b[0]);
    wire->write(b[1]);
    wire->write(b[2]);
    wire->write(b[3]);
    wire->endTransmission();
}
