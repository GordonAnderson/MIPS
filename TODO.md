# MIPS Project Roadmap

## Bugs
- [ ] RF driver 1 channel 2 PWM fre is 83KHz, not 50KHz
- [ ] Check RF driver 2

## Functional tests
- [ ] Fully test the platformIO version of MIPS
- [ ] Test serial watchdog, needs to reboot system on loss of comms activity

## Refactoring tasks
- [ ] No MIPS.h, do I need to add this file, ask Claude
- [ ] Change all module drivers to allocate there structures, saves ram
- [ ] Move DCbias serial commands from serial.cpp to DCbias.cpp
- [ ] Move RFdriver serial commands from serial.cpp to RFdriver.cpp
- [ ] Move ESI serial commands from serial.cpp to ESI.cpp
- [ ] Move ARB serial commands from serial.cpp to ARB.cpp
- [ ] Move DAC serial commands from serial.cpp to DAC.cpp
- [ ] Move Filament serial commands from serial.cpp to Filament.cpp
- [ ] Move Twave serial commands from serial.cpp to Twave.cpp
- [ ] Move Table serial commands from serial.cpp to Table.cpp
- [ ] Move WiFi serial commands from serial.cpp to WiFi.cpp
- [ ] Move ethernet serial commands from serial.cpp to ethernet.cpp
- [ ] Consider splitting MIPS.cpp — it is a very large file; the hardware init, main loop, and system commands could be separated  

## Claude tasks
- [ ] Remove dead code
- [ ] Refactor and comment menu.cpp/menu.h
- [ ] Refactor and comment dialog.cpp/dialog.h
- [ ] Refactor and comment serial.cpp/serial.h, after the commands are moved
- [ ] Refactor and comment TWIext.cpp/TWIext.h

## Structural/maintenance                        
- [ ] Add a MIPS.h (decide and track it)                                                                             
- [ ] Review and prune dead/commented-out code (there are several // blocks that look like old experiments, e.g. in WiFi_init, ethernet.cpp)                          
- [ ] Audit #include order and remove redundant includes now that files are .cpp 

## Documentation                                        
- [ ] Document the Variants.h flag combinations that are actually tested/supported 
- [X] Add a brief architecture overview (module list, TWI address map, serial command routing) 
                                                            
## New features
- [X] Add the help file creator python app to this build system
- [ ] Add RFamp/quad surrport for rev 3.0 hadware

