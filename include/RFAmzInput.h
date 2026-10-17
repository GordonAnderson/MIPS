#ifndef RFAMZINPUT_H_
#define RFAMZINPUT_H_
//
// RFAmzInput: set the RF amp / QUAD m/z from a MIPS ADC input, applied by a digital input strobe.
// See RFAmzInput.cpp.
//
// Called once per RF amp board from RFA_init.
void RFAmzInput_init(int8_t Board);
// Discards any strobe that is waiting to be applied. QSCAN calls this when it finishes so a strobe that came in
// during the scan, while the threads were not running, is not applied afterwards.
void RFAmzInputClear(void);

#endif
