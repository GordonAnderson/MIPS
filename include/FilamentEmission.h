#ifndef FILAMENTEMISSION_H_
#define FILAMENTEMISSION_H_
//
// FilamentEmission: closed loop emission current regulation for the filament module (Rev 3 and 4, which
// measure the emission current on the module). See FilamentEmission.cpp and FilamentEmissionCore.h.
//
// Called once per board from the end of Filament_init.
void FilamentEmission_init(int8_t Board);

#endif
