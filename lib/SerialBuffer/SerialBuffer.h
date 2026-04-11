#include <Arduino.h>
#include <inttypes.h>
#include <Wire.h>

#define SB_SIZE 512

class SerialBuffer: public Stream
{
public:
  SerialBuffer();
  ~SerialBuffer();
  void begin();
  void begin(TwoWire *twi, uint8_t add);
	virtual size_t write(uint8_t);
	virtual size_t write(const uint8_t *, size_t);
	virtual int  available(void);
	virtual int  read(void);
	virtual int  peek(void);
	virtual void flush(void);
	virtual void clear(void);
private:
  uint8_t  buf[SB_SIZE];
  uint16_t head;
  uint16_t tail;
  uint16_t sbsize;
  TwoWire  *wire;
  uint8_t  twiadd;
};