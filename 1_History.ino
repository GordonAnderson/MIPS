//
// MIPS version history
//
//    V1.0, January 13, 2015
//      1.) Orginal release
//    V1.1, January 25, 2015
//      1.) Cleaned up the code for the external trigger and tested. I added a ISR to disable the external trigger after it happens
//          so the system will not be re-triggerable
//      2.) I updated the DCbias board trip logic so it will turn off the outputs on the +- 50 volt board
//      3.) Fixed a minor display bug in the case where two DC bias board are in the system
//    V1.2, January 28, 2015
//      1.) Fixed a table bug that prevented table nesting in the case where
//          to channel entries are present just a nested table. Below is an example of a table
//          that did not work:
//              STBLDAT;0:[1:2,163:[2:2,0:3:0,326:3:300,815:3:0],3260:3:300,3423:3:0];
//    V1.3, January 30, 2015
//      1.) Fixed a number of bugs that prevented two DC bias modules from working in one MIPS system.
//    V1.4, February 3, 2015
//      1.) Added a delay to hold all DC bias DAC at zero until power is up and stable for 1 sec. The offset driver was
//          latching up on power up when set at 250 volts.
//      2.) Updated the power on voltage test to average the voltage 100 times. It has a looping problem sometimes that
//          I have not solved.
//      3.) Fix a table bug when 16 DC bias channels are present, this was a board select but that is now fixed.
//    V1.5, February 9, 2015
//      1.) Added support and commands for multiple tables and table advanceing.
//      2.) Added initial FAIMS module support.
//      3.) Fixed a bug in the serial ring buffer, I was using unsigned chars for ringer buffer pointers!
//    V1.6, March 18, 2015
//      1.) Added arc detection to FAIMS
//    V1.7, March 26, 2015
//      1.) Fixed bugs in the DC cv scanning on FAIMS module.
//      2.) Added the ESI control module code.
//    V1.8, April 3, 2015
//      1.) Fixed a couple ESI module bugs.
//      2.) Fixed table system bugs.
//          a.) Can not set a count of 0, is seen it will be incremented to 1.
//          b.) Tried to add a dialog box when in the table mode but this interfered with the SPI
//              interface needed in table processing, code was removed and it can be seen in tabledialog.cpp
//      3.) Added table voltage value edit function.
//      4.) Added serial support for bluetooth module. The system will now automatically switch back and forth between
//          serialUSB and serial. The bluetooth interface has problems with long strings, likely needs flow control.
//          Adding 3mSec of delay between transmitted characters at 9600 baud fixes the problem.
//      5.) Added the macro menu to MIPS config. This allows selecting and playing a macro and defining a
//          macro to play at startup
//      6.) Updated the Twave system to support the rev 2 module
//    V1.9, May 29, 2015
//      1.) Fixed the SendACK and SendNAK macros to end with \n\r instead of just \n, this is consistant with all other
//          serial responces and AMPS.
//      2.) Redesiged FAIMS to support field driven FAIMS option, not tested yet
//      3.) Updated the tack scheduler code so the execution time of the process is not added to the reoccurance time.
//    V1.10, June 6, 2015
//      1.) Fixed the bug with table commands containing 0 values
//      2.) Fixed a few minor serial commands
//      3.) Fixed disk io problems that happen with DC bias cards are present
//      4.) Upgraded to Arduino IDE version 1.6.4
//      5.) There is a problem with the SD interface when using multiple SPI CS options on the DUE. The only way I can make
//          the macro recording and work is to only record the macro in record mode and not execute the commands during macro
//          recording. The macro functions were upgraded in this version.
//    V1.11, June 9, 2015
//      1.) Fixed the FAIMS timer problems and on time problems
//      2.) Debugged the field driven FAIMS supplies
//    V1.12, June 18, 2015
//      1.) Added all the serial commands for the TWAVE module
//      3.) Fixed the table issue with a count of zero, I still think there are a few issues to resolve
//      2.) This verions was programmed into Matt Bush's system and Jim Bruce's system
//    V1.13, July 1, 2015
//      1.) Fixed FAIMS timing bugs and added the support for temp and pressure sensor
//      2.) Added the serial port reset function
//    V1.14, July 7, 2015
//      1.) Fixed a frequency bug with Twave rev 2 and added internal clock mode.
//    V1.15, July 20, 2015
//      1.) Added the analog input module based on the Adafruit ADS1115 boards, 8 channels max.
//      2.) Updated the thread controller to support names.
//      3.) Added the THREADS command to display the running threads and there last run times.
//      4.) Updated the RFdriver and DCbias modules to improve the run time and screen updates.
//      5.) Add the following new features to the RFdriver:
//          a.) Gate input selection and logic level
//          b.) Automatic control mode
//          c.) Created DIhandler class to support a.)
//      6.) Added the 750 volt DCbias board option
//    V1.16, July 26, 2015
//      1.) Added support for the 2 channel Filament module.
//      2.) Added watchdog timer support
//      3.) updated PWM to 12 bit operation
//   V1.17, August 25, 2015
//      1.) Fixed a number of minor bugs in the filament driver code
//   V1.18, August 26, 2015
//      1.) Fixed CV scan parameter limit bug in FAIMS code
//   V1.19, September 6, 2015
//      1.) Update The FAIMS module:
//          -- Added output level locking
//          -- Updated the display processing
//          -- Added commands to support calibration
//          -- Enabled the pressure and temp compensation
//   V1.20, October 13, 2015
//      1.) Added delay in the sequence loading function for Twave. The function failed at low frequencies.
//   V1.21, November 6, 2015
//      1.) Added support for ARB module
//   V1.22, December 19,2015
//      1.) Added WiFi support with ESP8266 module from Adafruit
//      2.) Added support for Rev 3 of Twave module
//   V1.23, December 31, 2015
//      1.) Updated WiFi interface
//      2.) Fixes Twave serial command errors
//      3.) Updated the test mode to support Twave
//      4.) Allow and valid EEPROM address for the Twave board, this ability will be migrated to all modules
//   V1.24, January 3, 2016
//      1.) Fixed a number of Twave module bugs and added support for two Twave modules
//      2.) Added the GNAME and SNAME commands.
//   V1.25, January 4, 2016
//      1.) Removed delay from the ISR function in Twave, this was causing it to crash.
//   V1.26, January 6, 2016
//      1.) Fixed a table bug that was allowing a table to be retriggered.
//      2.) Added a ONCe option to the SMOD command so that a table will only play one time them the table mode will exit
//   V1.27, January 22, 2016
//      1.) Added the cycling feaature to the Filament control module.
//      2.) Started the Compressor function for Twave.
//   V1.28, February 2, 2016
//      1.) Updated the compressor code, increased the max delay times to 90 mS.
//   V1.29, Feruary 11, 2016
//      1.) Updated startup code to require holding the button to enable default parameter startup.
//      2.) Updated DCbias moudule to support one offset parameter for both modules.
//      3.) Performed a lot of performance optomization on the DAC SPI for DC bias module,
//          performance increased to about 5 to 7 uS per channel for update. Discoverted the
//          digital IO is very slow on DUE so I used diret register access.
//      4.) All DAC and DIO SPI are now interrupt safe with SPI in an interrupt
//      5.) Added FAIMS arc detection sensitvity adjustment
//      6.) Improved the logic of DIO TWI re-talking in the event of error
//      7.0 Fixed FAIMS DC bias offset error.
//  V1.30, February 17, 2016
//      1.) Added the DCbias report all functions for setpoints and readbacks. GDCBALL and GDCBALLV.
//      2.) Added the RF driver report all function. It reports, freq, VPP+, VPP- for each rf channel.
//      3.) Added the TWI acquire and release system with queued function calling, or post use calling.
//      4.) Made display SPI interrupt safe, so all SPI should now me interrupt safe.
//  V1.31, February 22, 2016
//      1.) Fixed a table bug, this caused an error "xx]" where "xx:]" was ok, now both are ok.
//  V1.32, February 25, 2016
//      1.) Added echo mode to host communications.
//      2.) Added WiFi enable command.
//  V1.33, March 3, 2016
//      1.) The dual Twave in compressor mode is not compatiable with the RF driver. If the RF driver is installed
//          it must be at board address A. If the drive values are not adjusted then it seems to exsit but its values
//          must not be changed. The Twave compressor uses a software generated clock using a ISR and taxes the system.
//      2.) Removed the interrupt masking in the display driver because in caused compressor mode time jitter.
//      3.) Moved timers to resolve conflict with compressor
//  V1.34, March 11, 2016
//      1.) FAIMS updates to disable tune above 30% drive
//      2.) Cleaned up a few minor issues in the FAIMS code
//      3.) Only update the Twave DACs if the values change
//  V1.35, March 13, 2016
//      1.) Updated DCbias to only update on change and to update independed of LDAC
//      2.) Updated DCbias to only test for readback error in Local mode
//      3.) Enabled tasks in table mode, TBLTSKENA,TRUE command enables
//      4.) Increased the number of DIhandelers, was running out and causing issues with dual Twave compressor
//      5.) Updated the TWI queued function routine to allow up to 5 functions to be queued
//      6.) Fixed the USB isolator problem by changing the USB driver source code. The buffer size parameter
//          was changed from 512 to 64 in function _cdcinterface in file CDC.cpp.
//  V1.36, March 27, 2016
//      1.) Added the ADC command to read and report ADC inputs
//      2.) Updated Trigger output to support width in uS
//      3.) Added trigger out to signal table systax, is lower case t and the parameter is width in uS
//  V1.37, April 8, 2016
//      1.) Added RF driver rev 2 to support the new smaller RF heads with the linear tech level sensors.
//      2.) Updated command processor to allow int data input
//      3.) Allow user to set inter table delay in mS. Set default from 10 to 4
//  V1.38, April 13, 2016
//      1.) Fixed a table bug in external trigger mode where the system would stop accepting trigger, this was a very random event.
//      2.) Fixed a false voltgae error problem when exiting table mode and exiting calibration mode.
//  V1.39, April 18, 2016
//      1.) Initial support for rev 3.0 controller. This version has the second output port on its own strobe or LDAC, this is jumper selectable.
//          Initial support allows back compatibility but does not exploied the capability. Need additional upgrates, this was a quick fix.
//  V1.40, April 20, 2016
//      1.) Moved the WiFi enable to the MIPS data structure.
//      2.) Added command to allow changing a time point count value in a loaded and executing table.
//      3.) LED control on button, OFF, and select a colors for ON, and flashing
//                LEDOVRD,<TRUE or FALSE> this will override the default LED driver
//                LED,<value> this is a bit mask (integer) with the following bit definitions:
//                bit       function
//                 0          RED,   1=ON
//                 1          BLUE,  1=ON
//                 2          GREEN, 1=ON
//                 3          Blink if set to 1
//      - Need the following:
//          a.) Clean up the code for the Rev 3.0 controller and add the Rev 3.0 specifics.
//          b.) Figure out why the SSRs all flash on when first powered up. Need resistor to ground on strobe line, 1K
//          c.) Don't update the working data structures if the display is not selecting the module.
//          d.) Change RF frequency update logic to reduce the drive to 0 before changing the frequency.
//  V1.41, April, 27 2016
//      1.) Worked at Mike's lab and fixed a number of issues
//          - added flag to avoid reading ADCs if the tables values just changed, noise is induced in this case.
//          - Used the same flag for RF level readback
//          - needed a small delay in the table processing loop when the tasks are enabled.
//      2.) When the power supply is off the system returns the offset voltage, fixed this bug.
//      3.) Added a clock generation mode, supported commands:
//              SWIDTH      Set pulse width in microseconds
//              GWIDTH      Get pulse width in microseconds
//              SFREQ       Set frequency in Hz
//              GFREQ       Get frequency in Hz
//              BURST,num   num = number of clocks to generate. num=-1 to run forever and 0 to stop without changing stored num
//          Also able to trigger from the table. Add b command where parameter is the number of pulses
//      4.) Need to add support for offset readback and checking. This needs to be an option and only used in cases where a channel is
//          avaliable for readback. An output channel can be repurposed for offset readback.
//          - To enable this capability move the offset HV opamp to the second board and
//            also add a readback module in channel 8 slot. Jumper the optput for this channel 8 (actuall 16)
//            to the float input pin.
//          - Need to add a flag for this mode, use DCBOFFRBENA,TRUE or FALSE. This is DCbias offset readback enable.
//            Always used last channel for readback.
//      5.) Need to connect output enable to Arduino output pin to keep outputs disabled during initalization. Two options for this
//          depending on MIPS controller hardware revision.
//          - For revsion 2.2 and older there is a hardware bug that connects the buffered input in Arduino pin 46 (input W on MIPS)
//            to the output buffer enable. On these version cut the trace from IC6 pin 9 and program pin 46 as output, also remove R16
//            pull down. After initial set up drive 46 low.
//          - For revision 3.0, there are two pull downs, R16 for IC5 and R20 for IC4. To remove power up glitching remove the pull down(s)
//          - and jumper Arduino pin 44 to the output enable you wish to deglitch.
//      6.) Added Table stop command, TBLSTOP.
//  V1.42, April, 29 2016
//      1.) Fixed bug that came from the ADC resolution increase to 12 bit. This caused the power off detection to fail.
//      2.) Added power up and power down output reset logic
//  V1.43, May, 5 2016
//      1.) Added display enable command.
//      2.) Added watch dog timer kick in the process serial loop
//      3.) Added the shutter control for Mike Belov's system. This is hard coded and still needs work to make
//          a generic solution.
//  V1.44, May 9, 2016
//      1.) Updated filament cycle mode to accept 0 as forever in the number of cycles.
//      2.) Upded the Twave max order to 127
//      3.) Added order = 0 to = stop
//  V1.45, May15, 2016
//      1.) Removed shutter control commands and logic.
//      2.) Added DI port change reporting
//      3.) Added DI to DO port mirroring
//  V1.46, May 19, 2016
//      1.) Fixed a bug in Twave order 0 compressor mode
//  V1.47, May 21, 2016
//      1.) Add new table driven compressor mode
//      2.) Added all compressor host commands
//  V1.48, June 6, 2016
//      1.) Updated the DIO special operations and the table trigger to use the DIhandler object.
//          This allows the two capabilities to share an input channel.
//  V1.49, June 17, 2016
//      1.) Pulse output A when the scan is started to 10mS.
//  V1.50, June 19, 2016
//      1.) Added SOFTLDAC command to allow forcing software LDAC reguardless of MIPS version.
//      2.) Added Twave compressor commands to support voltage and time setting in compression table. Only support whole numbers at this point.
//          V,v,c,n,t are new commands.
//  V1.51, June 20, 2016
//      1.) Fixed a bug that prevented you from changing the frequency of a running clock.
//  V1.52, June 23, 2016 (work in progress)
//      1.) Table timing improvements. Reduced the minimum pulse width from 40uS to 12uS
//          - Moved the DAC updates inline
//          - Preprocess the DAC data so it only needs to load at run time
//          - Updated the functions that allow changing a value at table run time
//          - Need to test!
//      2.) Fixed a bug in the compressor table voltage setting. Board selection was not being done.
//      3.) Added flush after command processor in table mode command processing.
//      4.) Changed all the write function to println in table reporting.
//      5.) Added command to turn off table responses.
//      6.) Fixed command processor so message processing only happens with a complete message in buffer.
//  V1.53, June 29, 2016
//      1.) Finished testing and debugging the Fast table mode and updated the system for filament systems.
//          This version is with the FAST_TABLE mode enabled. Can get minimum pulse widths on order of 9 uS.
//  V1.54, July 2, 2016
//      1.) Added the SDCBDELTA command.
//      2.) Added 'F' frequency command to multi-pass compressor
//      3.) Added looping to multi-pass compressor, ...[...]x... where x is the loop count
//      4.) Added bias current monitoring capability to Filament module. Also added the serial commands to set
//          bias sense resistor, use 10k resistor on channel 8 of bias module.
//      5.) Made all the delay values in twave 100 seconds maximum
//      6.) Added the frezze mode to the compressor after a delay time. Do this is table as a starting point.
//      7.) Added the ehernet adapter code.
//      8.) Increased Twave maximum order to 255
//      --Need to add the following to this rev
//          a.) DMA SPI interface in table code
//          b.) Optomize the DIO in table mode, some of this is done
//  V1.55, July 30, 2016
//      1.) Added updates to support new rev 2.0 ARB module. Added the new ARB mode supporting pulse generation.
//      2.) Added ARB serial commands.
//      3.) Added the Twave multi-pass compressor switch control
//  V1.56, August 2, 2016
//      1.) Protocol error in the get filament current command was fixed.
//      2.) Updated the ARB to support rev 2.0 and the new ARB mode.
//  V1.57, August 8 2016
//      1.) Added the M0 and M1 command to twave compressor table, M0 is normal mode, M1 applies TW2 voltage
//          to TW1 in normal mode. this only happens in compressor table mode
//  V1.58, August 9 2016
//      1.) Added the M2 mode to the twave compressor, this applies TW2 voltage levels on TW1 in compress mode,
//          this only happens in the compressor table mode.
//  V1.59, August 17, 2016
//      1.) Updated WiFi to support serial port 0 or 1.
//  V1.60, September 2, 2016
//      1.) Added the table external clock EXTS mode for a hardware bug work a round at matt's lab in seattle.
//      2.) Added the EXTN for negative edge command but its not yet fully implementd.
//  V1.61, September 5, 2016
//      1.) Added the EXT and ETXN clock edge inversion logic
//      2.) Fixed a bug in the debounce logic for the EXTS clock mode of the table function.
//  V1.62, September 11, 2016
//      1.) Fixed the ext trigger function so it will work in the EXTS clock mode
//  V1.63, September 13, 2016
//      1.) Fixed a table bug that did not allow channels 9 through 15 to output
//  V1.64, September 29, 2016
//      1.) Fixed a table logic bug that was introduced with the table performance improvements. This bug caused the time point 0
//          values not to update if you are also changing digital outputs.
//  V1.65, October 2, 2016
//      1.) Added About command to report system parameters
//      2.) Added command to allow setting module rev level
//      3.) Added About menu option
//      3.) Upgrade the FAIMS module to support Rev 3 (this is a work in progress)
//          - New clock mode / setup
//          - Update the delay adjustment
//  V1.66, October 6, 2016
//      1.) Removed interrupt disables to improve clock generation in compressor enabled Twave systems
//      2.) Updated FAIMS driver to support hardware rev 3. Must set the board rev to 3
//  V1.67, October 14, 2016
//      1.) Improved the filtering on the bias current monitoring for the filament system
//  V1.68, October 15, 2016
//      1.) Added current ramp down on filament disable
//      2.) Added the DCbias profiles and profile commands including toggle between profiles
//      3.) Add disable offset command
//      4.) Added all the twave frequency sweep commands
//  V1.69, November 3, 2016
//      1.) Updated the ARB module driver with the following features:
//          - Added support for two modules
//          - Added External Sync
//          - Added compression logic
//          - Updated all of the host command and added host commands for compressor
//      2.) Added the initial drivers for the high order FAIMS controller
//      3.) Updated the table code to address the following issues
//          - Fixed an error that would not allow you to start a table with an initial delay
//          - Fixed a number of bugs when using external trigger to stat a table
//  V1.70, November 8, 2016. Election day!
//      1.) Moved all the sweep code into the generic compressor file
//      2.) Added voltage sweep to the sweep code
//      3.) Added the sweep functions to the ARB modules
//  V1.71, December 3, 2016
//      1.) Updated the Twave module driver to support rev 4.0 of the Twave hardware. This version
//          has a CPLD for clock generation. Rev 4 also supports lowering the minimum pulse voltage to 5 volts.
//      2.) Updated the Twave driver to only save a minimum pulse voltage of 15 volts.
//  V1.72, December 15, 2016
//      1.) Fixed a number of bugs in the RFdriver code to support using two RF driver boards in one system.
//  V1.73, December 20, 2016
//      1.) Fixed a bug that was introduced with rev 1.69. The table code update broke the table processing
//          for tables that start with 0:[, this is most tables!
//      2.) Re-enabled the serial port with buad rate set to 9600. variants.h has swith to turn it on and off
//          and set compile time baud rate
//      3.) Added DIO, DI, and DO to the get number of channels command
//      4.) Added remote e-stop input to FAIMS and HOFAIMS, input S is used and active high in both cases
//      5.) Added HOFAIMS power and drive limit code.
//      6.) Updated arc detector sensativity control for FAIMS.
//      7.) Added delayed triggering capability supporing ARB module only at this point
//  1.74, January 20, 2017
//      1.) Improved the RF driver closed loop control functions
//  1.75, Feburary 4, 2017
//      1.) Added common offset flag and function to the ARB
//      2.) Added FAIMS host commands to support the CV parking capability
//  1.76, Feburary 20, 2017
//      1.) Added RF driver auto tune function
//      2.) Added arc detect disable to FAIMS
//  1.77, Feburary 22, 2017
//      1.) Added ESI features:
//          a.) current limit and trip
//          b.) voltage limit
//          c.) voltage ramp up
//          d.) enable
//  1.78, March 1, 2017
//      1.) Added ESI rev 3 updates and tested. Rev 3 supports automatic polarity switching for the ESI voltage.
//          Rev 3.0 supports onlt the EMCO C series supplies
//      2.) Fixed bug in RF driver related to auto tune above channel 2
//  1.79, March 9, 2017
//      1.) Added the rev 2 mode for the filament driver. This allows connecting the two channels and then acting
//          as a single channel with current reveral capability. This has been developed and tested.
//      2.) Added a watch dog timer mode on serial traffic, if there is no serial traffic the system will disable
//          the filament driver channels.
//      3.) Added to the loss of power detection logic a filament power shutdown. The detection gives a total of 35 mS of advance
//          notice.
//      4.) Added the ability to display a graphics image on the display. and to startup with am image with the
//          display disabled.
//      5.) Added a tune complete message when auto tuning from a serial command.
//      6.) Added a retune command to make fine adjustments to the tuning, move in 1KHz steps at the current
//          power settings.
//      7.) Added Auto tune and retune response messages.
//      8.) Turning off the display now disables the button and reports to host button press and rotation.
//  1.80, March 20, 2017
//      1.) Rised the max filament power to 20 watts.
//      2.) Fixed type in auto tune response message.
//      3.) Added rf channel to response message for auto tune.
//      4.) Added logic to test for auti tune in process if commanded again
//  1.81, March 29, 2017
//      1.) Added the DAC module drivers
//  1.82, April 1, 2017
//      1.) Allow module EEPROM address to be user defined for all modules
//  1.83, April 26, 2017
//      1.) Added support for 4 ARB channels
//      2.) Added support for RF quad module (need to add serial commands)
//      3.) Updated ARB multi-pass table code to support and DIO for switches (not tested)
//      4.) Added support for up to 4 DC bias modules.
//  1.84, May 9, 2017
//      1.) Added .5 sec requirement of ESI overcurrent, this is to stop false detections
//      2.) Changed the DAC value reporting to include 3 places after the decimal point
//      3.) Add the voltage state linked list
//      4.) Make the mod described here: https://forum.voltivo.com/showthread.php?tid=8138
//          This fixed a bug causing MIPS to reboot when the USB cable is pulled while sending data from MIPS.
//  1.85, May 30, 2017
//      1.) Add the TRACE capability. This requires editing the linker file to free memory at the end of
//          sram. This memory saves the trace data and allows printing the buffer. Also added the STATUS
//          command to print the last reboot reason and the trace buffer.
//      2.) Added a host command to enable remote navigation of the UI, this supports operation without
//          a front panel control.
//  1.86, June 10, 2017
//      1.) Updated / finished the states capability and the segments. This code is used to support the
//          Thermo project but has general utility.
//      2.) Added rev 5 ot the Twave driver. This rev is designed to support the 500V Twave module and
//          adds monitoring to shutdown the system in the event of a voltage readback error.
//      3.) Tested and fixed a few bugs in the ARB 4 module driver code.
//  1.87, June 16, 2017
//      1.) Added SWEEP to the drlayed trigger function.
//      2.) Fixed a bug in the DIhandeler library.
//  1.88, July 1, 2017
//      1.) Minor bug fixes to the DCbias States and Segments code.
//      2.) Minor updates to improve performance of the DIhandeler.
//      3.) Updated the ARB common clock command to take affect without the need to reboot.
//      4.) Added the DCbias readback testing disable command.
//      5.) Added ESI over current alarm disable command.
//  1.89, July 4, 2017
//      1.) Added the TRIGOUT,FollowS command. This debounces the S input and reflects it to the trigger
//          output. This can then be used to clock the Q input for table mode.
//  1.90, July 19, 2017
//      1.) Added a number of SD card file management commands:
//          - DIR - returns a list of files and there size
//          - DEL,filename - allows you to delete a file
//          - GET and PUT functions that work with the MIPS host app to read and write files to SD
//      2.) Added module EEPROM saving and restore function to SD files
//          - SAVEMOD,filename,board (A or B),eeprom address (hex) - save the selected EEPROM to SD
//          - LOADMOD,filename,board (A or B),eeprom address (hex) - loads the selected file to EEPROM from SD
//          - SAVEALL - Saves all module's eeprom data to files, one file per eeprom found with the board
//                      and address coded into the name.
//          - LOADALL - Loads all the module's eeprom with files saved by SAVEALL
//      3.) Updated the display off command to clear the display when re-enabled.
//  1.91, July 24, 2017
//      1.) Added commands to allow reading and writing EEPROM module conifuration memory.
//      2.) Finished debug of all commands in V1.90 and enable file transfer to support
//          large files.
//      3.) Updated the ARB driver to support dual output boards and independant bias for each board.
//  1.92, July 29, 2017
//      1.) Fixed a bug in the ARB compression table code that caused non repeatable performance. Still
//          have work to do on the Twave compression table code
//      2.) Added table command to trigger the compression table, c:A for arb, c:T for twave.
//      3.) Added the ARB cramp command and functions.
//  1.93, August 10, 2017
//      1.) Fixed a number of bug in the Twave rev 4.0 system:
//          - CPLD updated to address several timing issues related to compression
//          - Fixed order 0 to equal infinity
//          - Gate function errors fixed
//          - Dir function errors fixed
//      2.) Updated the SD io functions to be interrupt safe
//  1.94, August 16, 2017
//      1.) Updated the ESI rev 3 module to ramp HV up and down and through 0 properly.
//      2.) Added the RFdriver calibration parameter entry function.
//  1.95, August 29, 2017
//      1.) Added UUID reporting, returns a 128 bit ID in hex format. This number is unique to the CPU.
//      2.) Fixed a table nesting looping bug.
//      3.) Minor update to serial processing to stop message intermingling.
//  1.96, September 3, 2017
//      1.) Fixed interrupt problem in ARB module
//      2.) Fixed DIhandeler setup issue that caused constant setup calls when input was NA (0)
//      3.) Saved the board select state in the table function
//      4.) Changed the RPT reporting to build the full string and send, sending in sections resulted
//          in missing chars on the MALDI app, reason is not clear but this update fixed the issue.
//      5.) Update the echo mode code to echo after the host sends enter
//      6.) Added the CrampOrder to ARB module
//      7.) Added Cramp and Cramp order commands to multi-pass compressor table, K and k
//  1.98, September 15, 2017
//      1.) Updated the ABOUT command to make it interrupt safe
//      2.) Added protection to set ARB compressor flag command
//  1.99, October 4, 2017
//      1.) fixed a bug in the macro record function that caused the system to reboot. also updated this function
//          so you can paste a macro commend into the terminal window.
//  1.100, October 13, 2017
//      1.) Fixed bug in the arb waveform edit function
//      2.) Updated the output settings when entering the shutdown mode
//  1.101, October 25, 2017
//      1.) Added waveform type control to the multi-pass compression table for ARB module
//      2.) Set initial RF drive to zero on startup for RF driver module.
//  1.102, November 2, 2017
//      1.) Fixed EXTS external trigger mode. Trigger had a lot of latency
//      2.) Added power supply turn on delay needed for 24 channel DCbias versions.
//  1.103, November 3, 2017
//      1.) Fixed bug in DCbias module that caused voltage trip to fail, this bug was introduced in 1.102 with code desiged
//          to delay DC power supply startup. Needed for 24 channel modules. Note, trip does not work with version 1.02!
//      2.) Added DCbias module power supply trip auto reset option
//  1.104, November 12, 2017
//      1.) Added serial commands for RF driver mode control
//  1.105, November 17, 2017
//      1.) Updated the RF driver for the Grid system to improve the closed loop control
//  1.106, November 25, 2017
//      1.) Updated the RF driver to support rev 4, this is the RF coil driver. The update is a bit of a hack
//          but this is a one off solution.
//      2.) Fixed a nasty table nesting bug that has been in the table code forever. This bug only happens
//          in complex nested tables.
//  1.107, December 2, 2017
//      1.) Fixed several bugs in table code that caused errors in nested loops.
//      2.) Fixed bug in MIPS timer getstatus function that caused missing interrupts.
//      3.) Added the ARB sync module software command
//      4.) Added the following ARB commands to the table system:
//              101 = aux channel 1
//              102 = aux channel 2
//              103 = aux channel 3
//              104 = aux channel 4
//              105 = offset channel 1a
//              106 = offset channel 1b
//              107 = offset channel 2a
//              108 = offset channel 2b
//  1.108, December 6, 2017
//      1.) Fixed a table bug that caused time 0 events to be missed in a repeat construct if it ended with digital command.
//          This was a SPI address register that did not properly represent the SPI address state thus causing the error.
//  1.109, December 16, 2017
//      1.) Added the backlight level control of the MIPS config menu, had to make config menu 2 pages
//      2.) Fixed table bug in end of loop processing introduced in 1.106 updates
//      3.) Adjusted the timer assigment to make the backlight work
//  1.110, December 18, 2017
//      1.) Fixed a bug in the MIPS timer class.
//  1.111, December 31, 2017
//      1.) Fixed a b ug the SD driver to allow compatability with SDHC cards
//  1.112, January 13, 2018
//      1.) Added channel number of voltage error message for DC bias module
//      2.) Added display auto intensity control
//      3.) Added ethernet adapter test command
//      4.) Updated the clock selection for the DCbias list functions
//  1.113, January 17, 2018
//      1.) Added support for second laser trigger using AUX trigger output. This used the delay trigger
//          function that can now be triggered by t and the AUX trigger will not exceed 60 Hz if using
//          the internal clock function. This is a very specific mod for MALID2 only.
//  1.114, January 19, 2018
//      1.) Added support for display type H8347_DSP. Updated the display driver to support both types of
//          displays. To use the H8347 short arduino due pin 49 to ground with a jumper.
//      2.) Added the CPUTEMP command to return the CPU temp in degrees C
//  1.115, January 31, 2018
//      1.) Fixed bug in S trigger on DCbias list function
//      2.) Added ability to loop forever on DCbias list function
//      3.) Added abort to DCbias infinate looping
//  1.116, Feburary 4, 2018
//      1.) Added the DC bias channel pulse capability
//  1.117, Feburary 14, 2018
//      1.) Fixed a bug in the DCbias list functions where the board select was not being defined.
//  1.118, February 19, 2018
//      1.) Added the SER1ENA,TRUE or FALSE command that will enable the serial1 port for general IO
//          Baud rate is fixed at 115200.
//      2.) Added the Gate voltage to the Twave module, does not work with rev 4.0
//  1.119, March 27, 2018
//      1.) Extended ARB compression order to 65535
//  1.120, April 7, 2018
//      1.) Added display disable command to MIPS saved parameters
//      2.) Added commands to for MIPS startup delay and display enable also copied to saved parameters
//      3.) Added parameter save to FLASH as well as SD. If no SD then FLASH is used, FLASH is always 
//          written with SD card
//  1.121, April 15, 2018
//      1.) Added module save to EEPROM capability, only RF is supported at this release
//  1.122, May 3, 2018
//      1.) Added the TWIRESET command
//      2.) Added thge TWI hardware switch to use TWI hardware to access ADC instead of bit banging
//      3.) Note; when TWI fails the communications timeout is long, the TWI_RESET function works and
//          needs to be automatically called, maybe include an automatic retry?
//  1.123, May 5, 2018
//      1.) Table bug, when software triggered there was a delay the time of the table before the table would start.
//  1.124, May 9, 2018
//      1.) Improved the TWI reset function and perform on both board channels
//      2.) Improved the ADC 7994 function to return error on TWI problem
//      3.) Added Save module commands for most modules
//  1.125, May 31, 2018
//      1.) Update the AD7998 and AD7994 ADC TWI routine to disable interrupts through most of the function.
//          this is needed to make it safe to pulse sequence generator interrupts that use the board select line.
//          This can cause problems for time tables with narrow events!
//  1.126, June 14, 2018
//      1.) Added the TWI reset when bus lock up is detected on AD5625 DAC.
//  1.127, June 18, 2018
//      1.) Added the delete all function, DEL,*
//      2.) Added the FAIMS step scan and scan serial commands
//      3.) Added FAIMS output level lock host commands
//  1.128, June 30, 2018
//      1.) Updated the RFamp module and added host commands. This version includes all the calculations 
//          to allow calculating RF and DC values given m/z etc.
//      2.) Added test of the signature when reading the default parameters from SD. If the signature does not match then the
//          file is not read.
//  1.129, July 17, 2018
//      1.) Added Q and q commands to TWAVE multi-pass compression table to allow changing the sequence. This was implemented
//          rev 4.0 and above.
//  1.130, July 24, 2018
//      1.) Added floating point number capability to the multipass compression tables.
//  1.131, July 25, 2018
//      1.) Fixed an table bug that only happens when using an external clock at frequencies below 1MHz. The bug caused the
//          initial time 0 table event to be randomly over written by the second event.
//      2.) Also fixed a display bug when writting waveform type to MIPS ARB from the host
//  1.132, July 26, 2018
//      1.) Fixed a bug in the table fix in version 1.131, need to wait for a full clock cycle.
//      2.) Added SerialUSB to Serial1 echo command
//      3.) Added command to allow user to define table retriggerablity. Default is not retriggerable
//  1.133, July 28, 2018
//      1.) Fixed the table bug that 1.132 tried to fix.
//  1.134, Aug 2, 2018
//      1.) Fixed a bug that caused the table commands not to be processed when using serial1 port.
//  1.135, Aug 20, 2018
//      1.) On the RFamp change the units on Ro to mm.
//      2.) Fixed a bug that would not let you enter closed loop mode on the RF driver MIPS UI
//  1.136, Sept 4, 2018
//      1.) Updated SD drivers, in Sd2Card.cpp, cardCommand function added additional clocks to let the card init
//  1.137, Sept 12, 2018
//      1.) Updated the ARB driver to support adjustment to the number of points per waveform cycle. (not finished yet,
//          the user definable waveforms still needs some development, also testing is needed).
//      2.) Need to add the capability for frequency sweeping, I did check and there is space for the variables in the ARB
//          data structure.
//  1.138, Sept 23, 2018
//      1.) Added the AUXOUT command for testing of the MALDI controller.
//      2.) Updated the delayed trigger AUXOUT, removed the 60Hz limit and use the frequency function's pulse width.
//      3.) Added support for using TWI interface to enet module.
//  1.139, Oct 9, 2018
//      1.) Added interlock capability
//      2.) Allow compression with seperate clocks on ARB
//      3.) Adjusted the ARB frequency calculation to reduce the error and never exceed 40KHz, 40KHz was 41.3KHz and its
//          now 39.7KHz
//  1.140, Oct 21, 2018
//      1.) Added programmable frequency limits for trigger out and Aux trigger out
//      2.) Added addition RFamp module commands
//      3.) Fixed the invert bug in the RFamp
//      4.) Fixed the DCbias menu selection bug in RFamp
//      5.) Fixed a bug in the ARB compresion table waveform type selection, w command
//  1.141, Nov 1, 2018
//      1.) Added enable command to RFamp module
//      2.) Changed Invert menu option to Normal in the RFamp module
//  1.142, Nov 15, 2018
//      1.) Updated for the new Arduino IDE. Had to add a switch for the old version, 1.6.5 because of
//          requestFrom changes in the TWI drivers. 
//  1.143, Dec 6, 2018
//      1.) Fixed bug in longStr command processing and retuned -1 to indicate string is processed.
//      2.) Added L and l commands to ARB compression table to control channel 3 and 4 voltage.
//      3.) Added ARB ramp rate and commands B,b,E,e to ARB compression table to control ramp rate.
//      4.) Added support for rev 3.1 of the power input modules, commands for on/off, voltage monitor
//          and power status.
//      5.) Fixed bug in ARB custom waveform download.
//      6.) Updated DCbias driver to allow disabling readback when using AD5593 ADC/DAC
//  1.144, Dec, 18 2018
//      1.) Fixed init issue with AUXTRIG output, it was initing high due to Arduino IDE upgrade
//      2.) DCbias board power supply readback was not using board select, fixed
//  1.145, Jan 15 2019
//      1.) Removed watchdog timer reset from the process serial command processing loop
//      2.) Added test of supply voltage and abort from table mode loop
//  1.146, Jan 16 2019
//      1.) Added uptime command.
//      2.) Fixed the watchdog timer, failed when I updated the IDE
//      3.) Added delay when HV power supply is enabled
//      4.) If a watchdog timer reset is detected on power up then a software reset is issued to make sure
//          the USB system is reset
//  1.147, Jan 19 2019
//      1.) The TWI queue capability was re-written to all queuing of functions with arguments. This cleaned
//          up the ARB compression code and supported added capability
//      2.) Added the m command to the ARB compressor to allow defining mode for any ARB channel
//      3.) Added the J command to the ARB compressor to allow defining the compresion order for any
//          ARB channel
//      4.) Fixed bug in the ARB compressor table 'F' command, hard limit at 40000
//  1.148, Jan 21 2019
//      1.) Fixed bug in the Get Table Value command, GTBLVLT, this bug was pretty old!
//      2.) Added capability to run the tasks when there is time in a table. Need at least 40mS to run
//          tasks. Updated the table code to allow running tasks when the table is waiting for an event, additionally if
//          in software trigger mode and waiting for a trigger command, process tasks. Enable and disable using
//          STBLTASKS, TRUE enable, FALSE by default. The scheme works by monitoring the counter value and comparing to
//          RA and RC to determine counts to next event. the SEXTFREQ command allos you to define an enternal frequency
//          needed to calculate avalible time. SEXTFREQ is zero by default. In the table processing loop, if we have
//          40 mS then call the next ready task.
//  1.149, Jan 28 2019
//      1.) Fixed a Filament bug that resulted in forward and reverse modes having different mode values.
//  1.150, Feb 5, 2019
//      1.) Added the MDIO command to look for a edge transistion and latch the data, add the RDIO command
//          to read the input state.
//      2.) Added calibraton function for RF level readbacks
//      3.) When enabling the DC bias supplies the system will now ramp the voltages up using 100 steps over
//          100 mS.
//      4.) The reset serial ports on the main menu now resets the USB drivers. If you have an open connection
//          performing this reset will break the connection
//  1.151, Feb 11, 2019
//      1.) Edited the USBCore.cpp and USBCore.h files and added timeout of potential lockup loops to prevent USB
//          lockups and thus watchdog timer resets.
//      2.) Updated the ARB compression table to support floating point values.
//      3.) Added watchdog timer reset to the RFdriver calibration loop.
//  1.152, Feb 15, 2019
//      1.) Fixed bug in ARB direction and improved performance. Dir change lantency is 17mS in Sin mode, 2mS
//          in pulse mode.
//      2.) Finished refining the SerialUSB auto recover code. The MIPS system will detect a SerialUSB error and 
//          reset the port.
//  1.153, Feb 21, 2019
//      1.) Fixed bug in serial auto reset that was introducted in version 1.151 and 1.152, this bug caused a comms slow
//          down. 
//  1.154, May 26, 2019
//      1.) Fixed bug in DCbias module, when useing readbacks on offset the AD5532 was read for every channel
//          instead of being read only one time
//      2.) Added the FAIMSfb module interface, includes electrometer code
//      3.) Added the TWIext file and header file to hold all the TWI related functions and developed generic
//          TWI higher level functions
//      4.) Added compile switches to Variants.h to enable turing off modules, also messaged at complie time and
//          signaled in version number
//      5.) Updated table code and cleaned up a number of issues
//      6.) Added ramping to table code
//      7.) A bug appeared when using a USB isolator on a PC, strings sent to the MIPS controller over 64 bytes
//          were dropping characters. Fixed this bug by changing the endpoint buffer size to 64 in the CDC.cpp 
//          file on lines 77 and 78.
//      9.) Mike's MALDI app failed due to the DTR requirment, removed this requirement
//      10.) Changed the DCbias driver readback to use the channel number index for the ADC array, look at ways to 
//           disable channels from testing.
//      11.) Moved all DIO function from Hardware to DIO.cpp
//      12.) Moved all file io from hardware to FILEIO
//      13.) Moved ADC functions from hardware file to ADCdvr file
//      14.) Added support for new RF driver module with on module M0 processor
//      15.) Update ARB sync command to work even if the arb channel is not setup for ext sync
//      16.) Updated the DCbias driver to allow two board on board select 0, also updated tabel code to support
//           this change. This allows 16 channels of DCbias with all channels using the same board select
//  1.155, July 3, 2019
//      1.) Fixed bug in the table code, fixed in table2. This bug was only happened when using the EXTS clock mode.
//          Ben in Bush's lab found the bug.
//  1.156, July 6, 2019
//      1.) Added the ARB sine wave generation capability for convention ARB mode, this is to support thermo project.
//      2.) Fix bug in RFamp driver that caused the second modules setpoint limit to be applied to both channels.
//  1.157, July 8, 2019
//      1.) Updated table 2 code to dynamically allocate table space
//      2.) Packed the table structures to provide more space, table 2 code
//
//      To do list for this version update
//          a.) Update ARB TWI commands to use the new function in TWIext file
//          b.) General ARB code clean up
//  1.158, August 22, 2019
//      1.) Updates to DCBlist functions
//          - Fixed command type for LSTATES, it was somehow changed to LTRIG
//          - Updated segment about function to no longer require and trigger to cause an abort
//          - Release the LDAC signal when segment processing finishes
//          - Fixed bug with multiple groups in one segment
//      2.) Added command to update all DCbias channels, SDCBUPDATE,TRUE
//      3.) Fixed bug in RFdriver2 that caused the wrong channel to auto tune
//      4.) Added FW rev 4 to ESI module, this allows two ESI outputs on rev 3.1 hardware
//  1.159, September 11, 2019 
//      1.) Added the use board select flag and commands. 
//      2.) Added support for the single board e-msion filament system
//  1.160, September 17, 2019 
//      1.) Allow PPP upto 128, still needs work on freq calculation and downloading waveforms. 
//      2.) Removed a number of bugs in the rev 4 filament code (used on rev 3.0 hardware). System
//          is fully calibrated and functional.
//      3.) Raised max power of RFdriver2 to 90 watts.
//      4.) Removed the interrupt disable in AD5593 code it fixed enet comm errors in filament system
//  1.161, October 23, 2019
//      1.) If no filament module is system then drive output pins used for filament enable to low state.
//      2.) Changed the dual electrometer zero function range to 1 to 5 pA.
//      3.) Increased the number of ramps in the table generator to 3.
//  1.162, November 4, 2019
//      1.) Fixed ramp function if statement missing set of ()
//      2.) Increased the table processing queue size to 8 from 5.
//  1.163, November 17, 2019
//      1.) Updated the read AD7998 to return error is there is a channel mismatch or any tranfer issues
//  1.164, November 30, 2019
//      1.) Added support for ESI hardware rev 4.1, use firmware rev 5.0
//  1.165, December 13, 2019
//      1.) Several updates to the command processing routine in the RFamp drivers
//      2.) Updated RFamp enable to also enable and disable the DCbias supply
//  1.166, December 16, 2019
//      1.) Added commands to the DCbias module to allow reading and setting DAC gain and offset.
//      2.) Fixed bug in ADC read vector function
//  1.167, Jan 11, 2020
//      1.) Added support for up to 6 ARB modules
//      2.) Added support for the ARB alternate waveform capability
//      3.) Redesigned the ARB drivers use of the DIhandlers to be more efficient
//  1.168, Jan 27, 2020
//      1.) Added the ADC change detection function and commands
//      2.) Added DCbias offset offset, and commands
//      3.) Added DCbias channel offset, mask, and commands
//      4.) Added commands to connect ADC to DCbias offset control and channel control
//      5.) Added command to disable USB powering controller, UOTGHS->UOTGHS_CTRL |= UOTGHS_CTRL_VBUSPO
//                                                        or  UOTGHS->UOTGHS_CTRL &= ~UOTGHS_CTRL_VBUSPO
//      6.) Added ADC value change reporting with gain control
//      7.) Added table commands that support changings table time values based on analog voltage change
//      8.) Added support for analog change to trigger table
//  1.169, Feb 22, 2020
//      1.) Added the A command for set aux voltage to the multipass compression table. A,channel,voltage
//          and a for negative voltages
//      2.) Updated the frequency setting to properly deal with ARBs with different PPP values
//      3.) Added commands for the ARB reverse direction application of a unique guard voltage
//  1.170, Feb 28, 2020
//      1.) Updated TWITALK to resolve an issue seen only on some systems. The signon message was long and
//          I think overrunning an internal buffer.
//  1.171, Mar 5, 2020
//      1.) Added support for ARB sync and compress line remapping, requires ARB2 rev 2.5
//  1.172, Mar 11, 2020
//      1.) Added limited backspace proceessing
//  1.173, Mar 14, 2020
//      1.) Added the following host commands
//          - Read an input pin, DREAD, returns HIGH or LOW
//          - Set an output pin, DWRITE, HIGH || LOW || PULSE (1uS) || SPLUSE (1mS)
//          - Set pin mode, DSET
//          - Add TWI1TALK for second I2C port
//  1.174, Mar 24, 2020
//      1.) Added updates to the DCB list capability
//          - Updated the LDAC capture after abort from a segment play
//          - Added report current 
//          - Added force trigger command
//  1.175, April 8, 2020
//      1.) Added the r, and s table commands that allow control of the compress and sync hardware lines
//          that are shared by all ARBs
//      2.) Fixed a bug in the table that would not allow the ARB module 1 AUX command to function
//  1.176, April 16, 2020
//      1.) Fixed a table bug when in external trigger mode and the trigger is shared with another function the
//          interrupts would fail after the first event.
//      2.) Added level det offset control to DCbias modules
//      3.) Updated the external trigger on table mode. The external trigger prep is now done as soon as the table 
//          finishes.
//  1.177, April 25,2020
//      1.) Added commands for input voltage, current, and power. Requires power input module 3.2
//      2.) Added command to allow serial watchdog to reset the serial port when there is no activity
//      3.) Updated the supplies command to also output current and also to send ACK
//      4.) Added no op command
//  1.178, May 3, 2020
//      1.) Fixed a long standing issue with the RF driver level readbacks, needed a delay in the bit banged 
//          TWI driver used for the AD7998
//  1.179, May 22, 2020
//      1.) Fixed bug in RFdriver gateing, only channel 1 of board A worked properly
//      2.) Added gatting off of the PLL clock generator when the RF channel is gated off
//      3.) Fixed bug with the warning LED when RF driver is gated off
//      4.) Added the TWI command functions
//  1.180, June 2, 2020
//      1.) Fixed a timing bug on the bit banged TWI interface, have a delay in the wrong place.
//  1.181, June 9, 2020
//      1.) Problem with RF driver auto tune, the algorithm finds a peak at half the resonate freq.
//          The result has always been at the low frequency limit of 400KHz, this problem only happened
//          after I reduced the minimum frequency to 400KHz. Changed routine to not use the lower limit value
//  1.182, June 11, 2020
//      1.) Improved the RF driver closed loop control. Removed the hunting issue with the RF level.
//      2.) Fixed a gating bug that cause the system to ignore mode changes
//  1.183, June 20, 2020
//      1.) Updated table number selection function to function while in table mode and waiting for a trigger.
//      2.) Added table trigger using the external level detection module and to read the lookup table to select
//          the table number to use.
//      3.) Added ARB command to use the level detection module to change the alternate waveform delay parameter.
//      4.) Added ARB command to use the level detection module to change the alternate waveform duration parameter.
//  1.184, July 29, 2020
//      1.) Added the gatting mode that used the level detection module to the ESI driver. This only works on the
//          firmware rev 4.0. Its desiged to work with hardware rev 3.2 with dual ESI outputs.
//      2.) Fixed a table bug when in external trigger mode and using multiple tables and the auto advance feature.
//          This bug was created with the 1.176 upgrade.
//      3.) Lowered the low freq limit to 100KHz on RFamp / QUAD
//  1.185, August 17, 2020
//      1.) Updated the RFamp/QUAD function to turn off the clock when the QUAD is disabled.
//      2.) Add the SWR to the main screen for tuning
//      3.) Need to add the ability to define the QUAD's dcbias channels
//  1.186, September 7, 2020
//      1.) Added ability to set the resolving DC bias channels for the RFamp module
//      2.) Updated the AD7998 ADC read function to allow the mux to stabalize without the need to read twice,
//          resulted in a 2x speed increase.
//  1.187, September 23, 2020
//      1.) Removed a bug that happened only with two RF drivers. Adjusting channel 3 would change channel 1 freq.
//          Also adjusting channel 4 would change channel 2 freq. This was a bug in the new gatting function.
//      2.) Added DCbias error testing hold off when pulse based Twave is reversed.
//  1.188, September 28,2020
//      1.) Fixed a bug in the channel mapping for the rev 3.0 and 4.0 emsion filament systems.
//  1.189, October 4, 2020
//      1.) Updated the RFamp DC voltage generation so the enable will set the values to zero and not disable 
//          the DC bias supply.
//  1.190, October 11,2020
//      1.) Added command to increase the power limit on the RFdiver. This is not yet implemented for RFdriver2
//  1.191, October 29, 2020
//      1.) Added auto calibration option to cal menu
//  1.192, November 15, 2020
//      1.) Added mask to invert the Digital inputs, this mask is saved on the MIPS controller SD.
//          This will need more work, it does not invert the digitial advanced functions using the DIhandler
//      2.) Added voltage limit, max DC voltage, for the DCbias module
//      3.) Fixed a bug that caused the system to crash if in ECHO mode and you issue a UUID command. Had to
//          flush the echo string after printing it!
//      4.) Added new feature. If your in boot image mode and you press the button the system will enable the
//          display.
//  1.193, December 4, 2020
//      1.) Updated ARB driver to support mixing rev 1 and rev 2 ARB modules.
//  1.194, December 6, 2020
//      1.) Added commands to read and set the DCbias readback trip percentage.
//      2.) Added command to read the RFamp (quad) gain range.
//      3.) Updated timing generator to support time sweeping. 
//  1.195, December 16, 2020
//      1.) Fixed minor bug in QUAD range changing, now it updates when the range is changed.
//  1.196, December 21, 2020
//      1.) Fixed a bug in the channel mapping function in DCBias module, caused the ramp function to fail
//  1.197, Jan 27, 2021
//      1.) Added the R option to the profile toggle command allowing a level on the R input to control the 
//          profile
//  1.198, Feb 5, 2021
//      1.) Updated ESI module for rev 5, changed calibration range to 10% and 50%, it was 0 to 50%
//      2.) Fixed ESI rev 5.0 range bug
//  1.199, Feb 21, 2021
//      1.) Updated FAIMS driver, added auto tune function for rev 3.0 and later hardware.
//          Still needs auto ratio adjust and real time phase adjustment.
//  1.200, Feb 28, 2021
//      1.) Finished development of auto tune function and added the host commands supporting
//          starting and aborting auto tune.
//      2.) Update FAIMS driver to reduce drive to 5% after a acr detection
//      3.) Level detection monitor command. Reports level interrupt and level detected. (not tested)
//      4.) Table timing test function.
//      5.) Improved the table timing.
//      6.) Increased the maximum m/z limit to 400000 in the host interface to the RF amp quad function
//  1.201, April 24, 2021
//      1.) Added real time clock function and commands to get and set time and date.
//      2.) Added DCbias module serial interface calibration function
//      3.) Added Fimament module rev 4 serial calibration function
//      4.) Added logging capability
//      5.) FAIMS updates:
//          - Added drive level control for tune mode, default 20%
//          - Added option to turn off enable disable control of curtian supply
//          - Added option to not auto dismiss arc detect message
//          - Added tune mode negative peak mode
//          - Fixed bug in curtian control that limited to +- 1000
//          - Ramp up drive level from external input
//          - Auto reset mode on arc detect
//          - Arc detect hold off on value change, decrease (this code is in place and looks ok so I raised the min level from .1KV to .5KV)
//
// We still have the problem with the AD7998 bit bang reading routine. Problem shows up on channel 2 of RF driver 
// at board select A. Using hardware TWI interface solves the issue.
//
//  BUG!, Twave rev 2 board require timer 6 to be used and not the current timer 7, the code need to be made
//        rev aware and adjust at run time. (Oct 28, 2016)
//
//  To dos / topics to think about:
//      1.) Update the display code to make it interrupt safe when ISR uses SPI, done but turned off for compressor
//      2.) Fix the DIO to allow command processing in table mode, this will require a pending update flag for
//          the DIO, only update if no pending update
//      3.) Consider re-implementing the table mode dialog box code. This should work after the SPI upgrades to
//          display driver from step 3.
//      4.) Add three point calibration to FAIMS high voltage levels
//      5.) Make the USB id text indicate GAACE
