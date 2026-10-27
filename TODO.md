# MIPS Project Roadmap

## Bugs
- [ ] RF driver 1 channel 2 PWM freq is 83KHz, not 50KHz
- [ ] Check RF driver 2
- [ ] ESI.cpp:621 `SaveESIsettings` writes board 1's EEPROM but sets `ESIarray[0]->Size` instead of `ESIarray[1]->Size` (copy-paste from the board-0 branch above it)
- [ ] ESI.cpp:872 `ESI_loop` has an author-flagged suspect comparison: `if(Setpoints[b][0] != 0)` — comment says "should be >= 0, not sure?" — needs verification against intended SetpointsR3 behavior
- [ ] ESI.cpp:754 `ESI_ADC_Control` dereferences `ESIarray[b]->Rev` without a NULL/-1 guard on `b` (`ESIchannel2board()` can return -1) — verify this can't actually happen, or add the guard
- [ ] ESI.cpp:542 `ESIgateChangeProcess` loop is `for(int brd=0;brd<1;brd++)` — only ever checks board 0; likely should be `brd<MAXESI`
- [ ] ESI.cpp:1306 `GetESIchannelMin` returns `0` for the dual-channel rev family instead of the negative channel's voltage limit, unlike `GetESIchannelMax` which branches per-channel correctly — confirm if intentional

### Module EEPROM / SD card commands (FILEIO.cpp, reviewed 2026-10-27)
- [ ] SAVEMOD/LOADMOD (`EEPROMtoSD`/`SDtoEEPROM`): validate the TWI address against `ModuleAddresses` (0x50,0x52,0x54,0x56,0x60,0x62) and check the `sscanf` hex parse (a non-hex token leaves `add` uninitialized). LOADMOD writes 512 bytes to any address it is given, a typo can write a DAC/ADC
- [ ] LOADMOD: check the module name in the file matches the name in the EEPROM before writing, with a FORCE option for a blank or corrupted EEPROM (recovery is a main use)
- [ ] LOADMOD: document that it changes only the EEPROM; send `RESTOREM,<type>` (or reboot) to apply it
- [ ] `SaveEEPROMtoSD`/`LoadEEPROMfromSD`: restore the board select on exit, as the SAVEM functions do
- [ ] `SaveEEPROMtoSD`: check the `file.write` count (a full SD card still ACKs); fix the stale "Remove the existing default.cfg file" comment
- [ ] LOADALL (`LoadAllfromSD`): `LoadEEPROMfromSD` calls `SD.begin` for every file while the root directory scan is open, which can break the scan; init the card once
- [ ] LOADALL: only write files for modules that are present and whose EEPROM name matches; respect `MIPSconfigData.UseBRDSEL` for B files (SAVEALL does); send one ACK, it sends `SendACKonly` at the start and `SendACK` at the end
- [ ] `EEPROMtoSerial` and its host-app partner (FILEIO.cpp:924, 968): address check accepts only 0x50–0x56 and rejects 0x60/0x62; `toupper(brd[0] != 'B')` has the bracket misplaced, so lowercase `b` is rejected

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
- [ ] ESI.cpp: cache `ESIchannel2board(chan)` in a local var in the serial command handlers that still call it multiple times per statement (e.g. `ValidESIvalue`, `SetESIchannel`, `GetESIchannel*`)
- [ ] ESI.cpp: remove leftover commented-out dead code (e.g. line 658 `memcpy` comment in `RestoreESIsettings`, lines ~1071-1074 rev 4.1/4.2 debug override comments in `ESI_loop`)
- [ ] ESI.cpp: replace magic calibration constants (e.g. `1.53`, `19600.0`, `1.3728`, `166.04`, `0.4`) with named constants documenting where they came from
- [ ] ESI.cpp: simplify the `esi`/`esich`/`esidata` triple state-bookkeeping (global working-copy singleton synced by hand across `ESI_loop`, `SaveESIsettings`, `RestoreESIsettings`, `UpdateESIdialog`) — root cause of bugs like the `Size` copy-paste above

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

