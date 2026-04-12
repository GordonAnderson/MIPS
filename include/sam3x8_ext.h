#ifndef SAM3X8_EXT_H_
#define SAM3X8_EXT_H_
// Forward declaration of _EEFC_ReadUniqueID which is defined in Serial.cpp
// but used by the GAACE debug library.
void _EEFC_ReadUniqueID(unsigned int *pdwUniqueID);

// Silence "SAM3X8 redefined" warning: the command-line -DSAM3X8 defines it as
// 1, but sam.h redefines it as a part_is_defined() expression. Undefine here
// (after any early guards have run) so sam.h can define it cleanly.
#undef SAM3X8
#endif
