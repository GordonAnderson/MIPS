/**
 * @file  WireServer.h
 * @brief Stateful I2C (TWI) helper that wraps the Arduino Wire library.
 *
 * WireServer simplifies sending and receiving typed values over I2C by
 * providing a set of strongly-typed read/write helpers built on top of an
 * Arduino TwoWire instance.
 *
 * Typical usage — command/response pattern
 * -----------------------------------------
 * Most peripheral protocols follow a two-step pattern:
 *   1. Send a command byte (or command + payload) to the peripheral.
 *   2. Read back a typed response.
 *
 * WireServer models this with:
 *   - sendCommand()  — opens a transmission, writes a command buffer, and
 *                      either closes the transmission immediately or defers
 *                      closing so the caller can append more bytes via the
 *                      Send* helpers before calling endTransaction().
 *   - Read* helpers  — request a fixed number of bytes from the last address
 *                      used by sendCommand() and unmarshal them into a typed
 *                      output variable.
 *   - Send* helpers  — write a typed value and close a deferred transmission.
 *                      These MUST only be called after sendCommand(..., true).
 *
 * Deferred-transmission workflow example
 * ----------------------------------------
 *   uint8_t cmd[] = { CMD_SET_VALUE };
 *   server.sendCommand(0x42, cmd, sizeof(cmd), true); // open, write cmd
 *   server.sendFloat(3.14f);                          // append, then close
 *
 * Non-deferred workflow example
 * --------------------------------
 *   uint8_t cmd[] = { CMD_READ_TEMP };
 *   server.sendCommand(0x42, cmd, sizeof(cmd));       // open, write, close
 *   float temp;
 *   server.readFloat(&temp);                          // request & unmarshal
 *
 * Byte order
 * ----------
 * All multi-byte values are transferred in little-endian order (LSB first),
 * which matches the native byte order of AVR and ARM Cortex-M devices.
 */

#ifndef WIRE_SERVER_H
#define WIRE_SERVER_H

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>

class WireServer
{
public:
    /**
     * Construct a WireServer using the default Wire instance.
     * Call setInterface() to substitute a different TwoWire object
     * (e.g. Wire1 on boards with multiple I2C peripherals).
     */
    WireServer();

    /**
     * Replace the underlying TwoWire interface.
     *
     * @param twi  Pointer to an already-initialised TwoWire instance.
     */
    void setInterface(TwoWire *twi);

    // ---------------------------------------------------------------------- //
    // Command transmission                                                    //
    // ---------------------------------------------------------------------- //

    /**
     * Write a command buffer to an I2C peripheral.
     *
     * Opens a transmission to @p addr, writes @p num bytes from @p buffer,
     * and — unless @p defer is true — closes the transmission immediately.
     *
     * When @p defer is true the transmission is left open so the caller can
     * append a typed payload with one of the Send* helpers, which will close
     * the transmission.  The target address is stored internally so that
     * subsequent Read* calls do not need it repeated.
     *
     * @param addr    7-bit I2C target address.  Also stored for Read* calls.
     * @param buffer  Bytes to send (command opcode and optional inline data).
     * @param num     Number of bytes in @p buffer (max 30 to stay within the
     *                Wire library's 32-byte buffer including address overhead).
     * @param defer   If true, leave the transmission open for a Send* call.
     * @return true on success; false if endTransmission() reports an error.
     *         Always returns true when @p defer is true (error checked later).
     */
    bool sendCommand(uint8_t addr, const uint8_t *buffer, uint8_t num,
                     bool defer = false);

    /**
     * Close a deferred transmission opened by sendCommand(..., true).
     * Call this instead of a Send* helper when no payload follows the command.
     *
     * @return true on success; false if endTransmission() reports an error.
     */
    bool endTransaction();

    // ---------------------------------------------------------------------- //
    // Typed read helpers                                                      //
    // ---------------------------------------------------------------------- //
    // Each helper requests the appropriate number of bytes from the address
    // stored by the most recent sendCommand() call and unmarshals them into
    // the caller-supplied output variable.
    //
    // Return value: true on success; false if insufficient bytes are available.
    // ---------------------------------------------------------------------- //

    /**
     * Read a 32-bit signed integer (4 bytes, little-endian).
     *
     * @param[out] value  Receives the unmarshalled value.
     * @return true on success; false if fewer than 4 bytes are available.
     */
    bool readInt32(int32_t *value);

    /**
     * Read a 16-bit signed integer (2 bytes, little-endian).
     *
     * @param[out] value  Receives the unmarshalled value.
     * @return true on success; false if fewer than 2 bytes are available.
     */
    bool readInt16(int16_t *value);

    /**
     * Read a single unsigned byte.
     *
     * @param[out] value  Receives the byte.
     * @return true on success; false if no byte is available.
     */
    bool readUint8(uint8_t *value);

    /**
     * Read a single signed byte.
     *
     * @param[out] value  Receives the byte.
     * @return true on success; false if no byte is available.
     */
    bool readInt8(int8_t *value);

    /**
     * Read a single boolean value (0 = false, non-zero = true).
     *
     * @param[out] value  Receives the boolean.
     * @return true on success; false if no byte is available.
     */
    bool readBool(bool *value);

    /**
     * Read a 32-bit IEEE 754 float (4 bytes, little-endian).
     *
     * @param[out] value  Receives the unmarshalled float.
     * @return true on success; false if fewer than 4 bytes are available.
     */
    bool readFloat(float *value);

    // ---------------------------------------------------------------------- //
    // Typed send helpers (for use after sendCommand(..., defer=true))         //
    // ---------------------------------------------------------------------- //
    // Each helper marshals its argument into bytes, writes them to the open
    // transmission, and calls endTransmission() to close it.
    //
    // IMPORTANT: these helpers MUST only be called after sendCommand() with
    // defer=true.  Calling them without an open transmission is undefined.
    // ---------------------------------------------------------------------- //

    /**
     * Append a boolean payload and close the deferred transmission.
     * @param value  The boolean to send (transmitted as a single byte).
     */
    void sendBool(bool value);

    /**
     * Append an unsigned byte payload and close the deferred transmission.
     * @param value  The byte to send.
     */
    void sendUint8(uint8_t value);

    /**
     * Append a 16-bit unsigned integer payload (little-endian) and close
     * the deferred transmission.
     * @param value  The value to send.
     */
    void sendUint16(uint16_t value);

    /**
     * Append a 24-bit (3 LSBs of a 32-bit int) payload (little-endian) and
     * close the deferred transmission.
     * @param value  The value to send; only the lowest 3 bytes are used.
     */
    void sendInt24(int32_t value);

    /**
     * Append a 32-bit IEEE 754 float payload (little-endian) and close the
     * deferred transmission.
     * @param value  The float to send.
     */
    void sendFloat(float value);

private:
    TwoWire *wire; ///< Underlying I2C interface.
    uint8_t  addr; ///< I2C address of the most recent sendCommand() target.
};

#endif // WIRE_SERVER_H
