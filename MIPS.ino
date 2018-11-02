//
// MIPS
//
// This sketch controls the MIPS system. This MIPS firmware uses the thread controller developed by
// Seidel Gomes, for details, go to https://github.com/ivanseidel/ArduinoThread. This is a round robin
// tasking system. Basically every module in MIPS is its own task. The modules are discoved on power up
// and there init functions are called. The modules then insert there menu option into the main system
// menu. There are a couple of additional threads defined in this main loop, one for the LED light
// flashing and a system thread that monitors power.
//
// The code in the modules does not block unless it is desired to stop all other tasks. Very little is
// done in ISRs with the exception of the table execution for pulse sequence generation.
//
// MIPS used ADCs and DACs with TWI and SPI interfaces. The TFT display is also SPI as if the SD card
// interface.
//
// Host computer communications is supported through both the native USB and serial ports. On power up
// it defaults to native USB and if serial traffic is seen it switches to the active port automatically.
//
// General Notes:
//  1.) When SAVE command is sent MIPS will save its configuration data structure to a file called
//      default.cfg on the SD card. This file is automatially loaded on startup.
//  2.) MIPS code uses a first order difference equation to filter the readback data. Here is the
//      transfer function of this filter:
//      Filter time constant is:
//      TC in seconds = 1/(sample rate is samples per sec * filter value) * 2 * pi
//  3.) The PWM output frequency needs to be increased from its default setting of 1000 Hz
//      to 50KHz for the RF driver module. The module used the following PWM output pins,
//      5 and 7 for board A and 6 and 8 for board B. This can be changed my editing the
//      variant.h file in the arduino source and changing two #defines:
//            #define PWM_FREQUENCY		50000
//            #define TC_FREQUENCY    50000
//      Also the PWM resolution is set to 10 bits by editing the following parameters in the
//      same file:
//            #define PWM_MAX_DUTY_CYCLE  1023
//            #define PWM_RESOLUTION      10
//            #define TC_MAX_DUTY_CYCLE   1023
//            #define TC_RESOLUTION       10
//      If the IDE is updated then these values need to also be updated. I am looking for
//      a cleaner way to do this.
//  4.) Use of Push button LEDs:
//        1.) When output voltages exceed 0 the blue LED will be on indicating output are on.
//        2.) If any output voltgae exceeds 50 volts the red LED will be on to provide warning.
//        3.) If any output exceeds 100 volts the red LED will flash.
//
// To do list: --------------------------------------------------------------------------------------
//   1.) Add three point calibration to FAIMS high voltage levels
//   2.) Make the USB id text indicate GAACE
//
//
// Revision history and notes *********************************************************************************
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
//      1.) Table bug, when software triggered they was a delay the time of the table before the table would start.
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
//  1.30, July 24, 2018
//      1.) Added floating point number capability to the multipass compression tables.
//  1.131, July 25, 2018
//      1.) Fixed an table bug that only happens when using an external clock at frequencies below 1MHz. The bug caused the
//          initial time 0 table event to be randomly over written by the second event.
//      2.) Also fixed a display bug when writting waveform type to MIPS ARM from the host
//  1.132, July 26, 2018
//      1.) Fixed a bug in the table fix in version 1.131, need to wait for a full clock cycle.
//      2.) Added SerialUSB to Serial1 echo command
//      3.) Added command to allow user to define table retriggerablity. Default in not retriggerable
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
//      1.) Updated the ARB driver to support adjustment os the number of point per waveform cycle. (not finished yet,
//          the user definable waveforms still needs some development, also testing is needed).
//      2.) Need to add the capability for frequency sweeping, I did check and there is space for thge variables in the ARB
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
//
//
//  BUG!, Twave rev 2 board require timer 6 to be used and not the current timer 7, the code need to be made
//        rev aware and adjust at run time. (Oct 28, 2016)
//
// Next version to dos:
//      1.) Update the display code to make it interupt safe when ISR uses SPI, done but turned off for compressor
//      2.) Fix the DIO to allow command processing in table mode, this will require a pending update flag for
//          the DIO, only update if no pending update
//      3.) Consider re-implementing the table mode dialog box code. This should work after the SPI upgrades to
//          display driver from step 3.
//
// Serial.println(); will print '\r' and '\n',
//
// Gordon Anderson
// GAA Custom Electronics, LLC
// gaa@owt.com
// 509.628.6851 (cell)
// 509.588.5410
//
#include <DueFlashStorage.h>
#include "SD.h"
#include "utility/Sd2card.h"
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9340.h"
#include <Adafruit_BMP085_U.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADS1015.h>
#include "Encoder.h"
#include <stdarg.h>
#include "Menu.h"
#include "Dialog.h"
#include "Hardware.h"
#include "Serial.h"
#include <Wire.h>
#include "AtomicBlock.h"
#include "DIO.h"
#include "Twave.h"
#include "DCbias.h"
#include "DCbiasList.h"
#include "FAIMS.h"
#include "ESI.h"
#include "ARB.h"
#include "HOFAIMS.h"
#include "Variants.h"
#include "Errors.h"
#include "Serial.h"
#include "ethernet.h"
#include <Thread.h>
#include <ThreadController.h>
#include <MIPStimer.h>
#include <DIhandler.h>

#pragma GCC optimize "-Os"

#if defined(__SAM3X8E__)
#undef __FlashStringHelper::F(string_literal)
#define F(string_literal) string_literal
#endif

#define UseWDT

// Define my names for the native USB driver
#define  USB_PRODUCT      "MIPS, native USB"
#define  USB_MANUFACTURER "GAA Custom Electronics, LLC"

DueFlashStorage dueFlashStorage;
const PROGMEM byte NonVolStorage[1000] = {};

bool Suspend = false;

// These values are defaulted to 1000 in Variant.h, The RFdriver needs this PWM
// frequency set to 50KHz for its adjustable power supply. Not sure if this overrides
// the values in the .h file?
#define PWM_FREQUENCY	    50000
#define TC_FREQUENCY      50000

bool NormalStartup = true;
bool SDcardPresent = false;

bool DisableDisplay = false;

bool LEDoverride = false;
int  LEDstate = 0;

uint32_t BrightTime=0;

const char Version[] PROGMEM = "Version 1.141, Nov 1, 2018";

// ThreadController that will control all threads
ThreadController control = ThreadController();
//MIPS Threads
Thread MIPSsystemThread = Thread();
Thread LEDThread        = Thread();

Encoder enc;
int encValue = 0;
bool ButtonPressed = false;
bool ButtonRotated = false;

bool EnableSerialNavigation = false;

extern Menu HardwareTestMenu;
extern DialogBox TwaveDialog;
extern DialogBox MIPSconfig;
extern DialogBox MIPSabout;
extern DialogBox ModuleConfig;
extern DialogBox MacroOptions;

#define MaxMainMenuEntries  16

void SerialPortReset(void);
void DisplayAbout(void);

char BoardAddress[20] = "";
char BoardName[20] = "";

// MIPS main menu
MenuEntry MainMenuEntries[MaxMainMenuEntries] = {
  {" MIPS Configuration", M_DIALOG, 0, 0, 0, NULL, &MIPSconfig, NULL, NULL},
  {" About this system",  M_DIALOG, 0, 0, 0, NULL, &MIPSabout, NULL, DisplayAbout},
  {" Reset serial port",  M_FUNCTION, 0, 0, 0, NULL, NULL, SerialPortReset, NULL},
  {""}
};

Menu MainMenu = {
  {"MIPS main menu", ILI9340_BLACK, ILI9340_WHITE, 2, 0, 0, 300, 220, B_DOUBLE, 12},
  M_SCROLLING, 0, 0, MainMenuEntries
};

extern DialogBoxEntry MIPSconfigEntriesP2[];

// MIPS configuration dialog box
DialogBoxEntry MIPSconfigEntries[] = {
  {" Controller rev"     , 0, 1, D_INT,    1, 10, 1, 21, false, "%2d", &MIPSconfigData.Rev, NULL, NULL},
  {" Config modules"     , 0, 2, D_DIALOG, 0, 0, 0, 0, false, NULL, &ModuleConfig, SetupModuleConfig, NULL},
  {" Startup delay"      , 0, 3, D_INT,    1, 100, 1, 20, false, "%3d", &MIPSconfigData.StartupDelay, NULL, NULL},
  {" Startup hold"       , 0, 4, D_YESNO,  0, 1, 1, 21, false, NULL, &MIPSconfigData.StartupHold, NULL, NULL},
  {" DCbias supply"      , 0, 5, D_ONOFF,  0, 1, 1, 20, false, NULL, &MIPSconfigData.PowerEnable, NULL, NULL},
  {" DCbias trip, %%FS"  , 0, 6, D_FLOAT,  0, 100, 0.1, 18, false, "%5.1f", &MIPSconfigData.VerrorThreshold, NULL, NULL},
  {" Reboot"             , 0, 7, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, Software_Reset, NULL},
  {" Backlight, %%"      , 0, 8, D_INT,      5, 100, 1, 20, false, "%3d", &MIPSconfigData.BackLight, NULL, SetBackLight},
  {" Next page"          , 0, 10, D_PAGE, 0, 0, 0, 0, false, NULL, MIPSconfigEntriesP2, NULL, NULL},
  {" Return to main menu", 0, 11, D_MENU, 0, 0, 0, 0, false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBoxEntry MIPSconfigEntriesP2[] = {
  {" Macro options"     , 0, 1,  D_DIALOG,   0, 0, 0, 0, false, NULL, &MacroOptions, SetupMacroOptions, NULL},
  {" Interlock input"   , 0, 2,  D_DI      , 0, 0, 2, 21, false, DIlist, &MIPSconfigData.InterlockIn, NULL, NULL},
  {" Interlock Output"  , 0, 3,  D_DO      , 0, 0, 2, 21, false, DOlist, &MIPSconfigData.InterlockOut, NULL, NULL},
  {" Save settings"     , 0, 9,  D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, SaveMIPSSettings, NULL},
  {" Restore settings"  , 0, 10, D_FUNCTION, 0, 0, 0, 0, false, NULL, NULL, RestoreMIPSSettings, NULL},
  {" First page"        , 0, 11, D_PAGE, 0, 0, 0, 0, false, NULL, MIPSconfigEntries, NULL, NULL},
  {NULL},
};

DialogBox MIPSconfig = {
  {"MIPS Configuration", ILI9340_BLACK, ILI9340_WHITE, 2, 0, 0, 300, 220, B_DOUBLE, 12},
  M_SCROLLING, 0, 0, false, MIPSconfigEntries
};

DialogBoxEntry MIPSaboutEntries[] = {
  {"   Return to main menu  ", 0, 11, D_MENU, 0, 0, 0, 0, false, NULL, &MainMenu, NULL, NULL},
  {NULL},
};

DialogBox MIPSabout = {
  {"About this system", ILI9340_BLACK, ILI9340_WHITE, 2, 0, 0, 300, 220, B_DOUBLE, 12},
  M_SCROLLING, 0, 0, false, MIPSaboutEntries
};

// MIPS module setup / config dialog box
DialogBoxEntry ModuleConfigEntries[] = {
  {" Board addr", 0, 3, D_LIST, 0, 0, 10, 12, false, BoardAddressList, BoardAddress, NULL, NULL},
  {" Board type", 0, 4, D_LIST, 0, 0, 10, 12, false, BoardVariantsNames, BoardName, NULL, NULL},
  {" Format!",    0, 6, D_FUNCTION, 0, 0, 0, 16, false, NULL, NULL, BoardFormat, NULL},
  {" Return to config menu", 0, 10, D_DIALOG, 0, 0, 0, 0, false, NULL, &MIPSconfig, NULL, NULL},
  {" Format Module's EEPROM", 0, 1, D_TITLE, 0, 0, 0, false, NULL, NULL, NULL, NULL},
  {NULL},
};

DialogBox ModuleConfig = {
  {"Module Configuration", ILI9340_BLACK, ILI9340_WHITE, 2, 0, 0, 300, 220, B_DOUBLE, 12},
  M_SCROLLING, 0, 0, false, ModuleConfigEntries
};

// Macro dialog box supporting the following
//  - Select and play a macro
//  - Select a macro to play at power up

char *MacroNameList = NULL;
char PlayMacro[20] = "";

DialogBoxEntry MacroEntries[] = {
  {" Play", 0, 2, D_LIST, 0, 0, 10, 12, false, MacroNameList, PlayMacro, NULL, UIplayMacro},
  {" Startup", 0, 3, D_LIST, 0, 0, 10, 12  , false, MacroNameList, MIPSconfigData.StartupMacro, NULL, NULL},
  {" Return to config menu", 0, 10, D_DIALOG, 0, 0, 0, 0, false, NULL, &MIPSconfig, NULL, NULL},
  {NULL},
};

DialogBox MacroOptions = {
  {"Macro options", ILI9340_BLACK, ILI9340_WHITE, 2, 0, 0, 300, 220, B_DOUBLE, 12},
  M_SCROLLING, 0, 0, false, MacroEntries
};

// This function is called before the macro menu loads.
void SetupMacroOptions(void)
{
  MacroNameList = MacroBuildList("NONE");
  MacroEntries[0].fmt = MacroNameList;
  MacroEntries[1].fmt = MacroNameList;
}

void SetBackLight(void)
{
  static bool inited = false;

  if (!inited)
  {
    inited = true;
    analogWriteResolution(12);
    pinMode(BACKLIGHT, OUTPUT);
  }
  analogWrite(BACKLIGHT, (MIPSconfigData.BackLight * 4095) / 100);
}

void SerialPortReset(void)
{
  SerialUSB.flush();
  SerialUSB.end();
  SerialInit();
}

// This function is called by the serial command processor. This function will report the following:
// MIPS version
// MIPS name
// List of modules, board, address, name, rev
void About(void)
{
  uint8_t addr;
  char    signature[100];
  int8_t  rev;
  int     iStat;

  SendACKonly;
  if (SerialMute) return;
  int b = SelectedBoard();
  // Report the system version and name
  serial->print("MIPS: ");
  serial->println(Version);
  serial->print("MIPS name: ");
  serial->println(MIPSconfigData.Name);
  if (MIPSconfigData.UseWiFi) serial->println("WiFi module enabled");
  if (EthernetPresent) serial->println("Ethernet module enabled");
  // Report all the modules and revisions
  serial->println("System modules:");
  serial->println("  Board,Address,Name,Rev");
  for (addr = 0x50; addr <= 0x56; addr  += 2)
  {
    // Set board select to A
    ENA_BRD_A;
    {
      AtomicBlock< Atomic_RestoreState > a_Block;
      iStat = ReadEEPROM(signature, addr, 0, 100);
    }
    if (iStat == 0)
    {
      serial->print("  a,");
      serial->print(addr, HEX);
      serial->print(",");
      serial->print(&signature[2]);
      serial->print(",");
      rev = signature[22];
      serial->println(rev);
    }
    // Set board select to B
    ENA_BRD_B;
    {
      AtomicBlock< Atomic_RestoreState > a_Block;
      iStat = ReadEEPROM(signature, addr, 0, 100);
    }
    if (iStat == 0)
    {
      serial->print("  b,");
      serial->print(addr, HEX);
      serial->print(",");
      serial->print(&signature[2]);
      serial->print(",");
      rev = signature[22];
      serial->println(rev);
    }
  }
  SelectBoard(b);
}

// This function is called after the About dialog box is created. This function
// needs to display all the about data.
void DisplayAbout(void)
{
  uint8_t addr;
  char    signature[100], buf[25];
  int y = 0;

  PrintDialog(&MIPSabout, 1, y++, "MIPS Version:");
  PrintDialog(&MIPSabout, 2, y++, (char *)&Version[8]);
  PrintDialog(&MIPSabout, 1, y, "MIPS Name:");
  PrintDialog(&MIPSabout, 11, y++, MIPSconfigData.Name);
  if (MIPSconfigData.UseWiFi) PrintDialog(&MIPSabout, 1, y++, "WiFi module enabled");
  if (EthernetPresent) PrintDialog(&MIPSabout, 1, y++, "Ethernet module enabled");
  PrintDialog(&MIPSabout, 1, y++, "System modules:");
  PrintDialog(&MIPSabout, 1, y++, "  Board,Addr,Name,Rev");
  for (addr = 0x50; addr <= 0x56; addr  += 2)
  {
    // Set board select to A
    ENA_BRD_A;
    if (ReadEEPROM(signature, addr, 0, 100) == 0)
    {
      sprintf(buf, "  a,%x,%s,%d", addr, &signature[2], signature[22]);
      PrintDialog(&MIPSabout, 1, y++, buf);
    }
    // Set board select to B
    ENA_BRD_B;
    if (ReadEEPROM(signature, addr, 0, 100) == 0)
    {
      sprintf(buf, "  b,%x,%s,%d", addr, &signature[2], signature[22]);
      PrintDialog(&MIPSabout, 1, y++, buf);
    }
  }
}

// This function is called by the serial command processor. This function will set a modules rev level to the value
// defined. All the parameters are pulled from the input ring buffer. The parameters are board,addr,rev
void SetModuleRev(void)
{
  char    *Token;
  char    *ptr;
  String  sToken;
  char    brd;
  uint8_t addr, rev;

  while (1)
  {
    // Read the board address
    GetToken(true);
    if ((Token = GetToken(true)) == NULL) break;
    brd = toupper(Token[0]);
    // Read the hex address
    GetToken(true);
    if ((Token = GetToken(true)) == NULL) break;
    addr = strtol(Token, &ptr, 16);
    // Read the target rev level
    GetToken(true);
    if ((Token = GetToken(true)) == NULL) break;
    sToken = Token;
    rev = sToken.toInt();
    // Validate the parameters
    if ((brd != 'A') && (brd != 'B')) break;
    if ((addr != 0x50) && (addr != 0x52) && (addr != 0x54) && (addr != 0x56)) break;
    // Write the rev number to EEPROM
    if (brd == 'A') ENA_BRD_A;
    else ENA_BRD_B;
    if (WriteEEPROM(&rev, addr, 22, 1) != 0)
    {
      SetErrorCode(ERR_EEPROMWRITE);
      SendNAK;
      return;
    }
    SendACK;
    return;
  }
  // If here then we had bad arguments!
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

// Called after macro is selected to play
void UIplayMacro(void)
{
  MacroPlay(PlayMacro, true);
}

// This function is called before the module configuration menu is loaded.
// This function fills the initial board address and name variables.
void SetupModuleConfig(void)
{
  sprintf(BoardAddress, "%s", GetEntry(BoardAddressList, 1));
  sprintf(BoardName, "%s", GetEntry(BoardVariantsNames, 1));
}

// This function is called from the host command processor and the arguments are in the
// serial input ring buffer on call. The arguments are case sesative and the same as the 
// MIPS UI configure menu format options, example usage:
//
// FORMAT,A 0x50,RFdrvA R1
void FormatEEPROM(void)
{
   char   *Token;

   while(1)
   {
     // Read the address
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     strcpy(BoardAddress, Token);
     // Read the board name
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     strcpy(BoardName,Token);
     GetToken(true);
     SendACKonly;
     if(BoardFormatEEPROM()) serial->println("Module formatted!");
     else serial->println("Unable to format");
     return;
   }
   // If here then we had bad arguments!
   SetErrorCode(ERR_BADARG);
   SendNAK;  
}

// This function is called by the UI to format a board's EEPROM.
void BoardFormat(void)
{
  if (BoardFormatEEPROM()) DisplayMessage("Module formatted!", 2000);
  else DisplayMessage("Unable to format!", 2000);
}

bool BoardFormatEEPROM(void)
{
  void *BoardConfigData;
  uint8_t addr;
  uint8_t brd;
  int CurrentBoard;

  // Scan the board number and board address from the BoardAddress string
  // Two board posibilities, A and B, and 4 addresses, 50,52,54, and 56.
  if (BoardAddress[0] == 'A') brd = 0;
  if (BoardAddress[0] == 'B') brd = 1;
  if (strcmp(&BoardAddress[2], "0x50") == 0) addr = 0x50;
  if (strcmp(&BoardAddress[2], "0x52") == 0) addr = 0x52;
  if (strcmp(&BoardAddress[2], "0x54") == 0) addr = 0x54;
  if (strcmp(&BoardAddress[2], "0x56") == 0) addr = 0x56;
  // Setup a pointer to the selected default data structure and get its size
  BoardConfigData = BoardVariants[FindInList(BoardVariantsNames, BoardName) - 1];
  // Write it to the selected board
  CurrentBoard = digitalRead(BRDSEL);  // Save current board select
  if (brd == 0) ENA_BRD_A;             // Select board
  else ENA_BRD_B;
  // Write to EEPROM
  if (WriteEEPROM(BoardConfigData, addr, 0, *(int16_t *)BoardConfigData) != 0)
  {
    digitalWrite(BRDSEL, CurrentBoard);  // Restore board select
    return false;
  }
  digitalWrite(BRDSEL, CurrentBoard);  // Restore board select
  return true;
}

void SaveMIPSSettings(void)
{

  if (SAVEparms("default.cfg") == 0) DisplayMessage("Parameters Saved!", 2000);
  else DisplayMessage("Error saving!", 2000);
}

void RestoreMIPSSettings(void)
{
  if (LOADparms("default.cfg")) DisplayMessage("Parameters Restored!", 2000);
  else DisplayMessage("Unable to Restore!", 2000);
}

// This function will add the menu entry to the MIPS main menu.
// This is used by modules as they init to add them selfs to
// the system.
void AddMainMenuEntry(MenuEntry *me);
void AddMainMenuEntry(MenuEntry *me)
{
  int   i;

  for (i = 0; i < MaxMainMenuEntries - 1; i++)
  {
    if (MainMenuEntries[i].Name[0] == 0)
    {
      memcpy(&(MainMenuEntries[i]), me, sizeof(MenuEntry));
      MainMenuEntries[i + 1].Name[0] = 0;
      return;
    }
  }
}

// Using software SPI is really not suggested, its incredibly slow
// Adafruit_ILI9340 tft = Adafruit_ILI9340(_cs, _dc, _mosi, _sclk, _rst, _miso);
// Use hardware SPI
Adafruit_ILI9340 tft = Adafruit_ILI9340(_cs, _dc, _rst);
//HX8347 tft = HX8347(_cs, _dc, _rst);

File myFile;

// This function is called when the button is changed or pressed
// to raise the intensity. If the intensity is less then 50% it is set
// to 50% and a timer is started, after 30 seconds the intensity is
// reduced to the user setting
void DisplayIntensity(void)
{
  if(MIPSconfigData.BackLight >= 50) return;
  analogWrite(BACKLIGHT, 2047);  // Set to 50%
  BrightTime = millis(); 
}

void encChanged(void)
{
  DisplayIntensity();
  if (DisableDisplay)
  {
    // If serial is enabled then send message
    if (!SerialMute) serial->println("Button rotated.");
    return;
  }
  ButtonRotated = true;
}

void encPB(void)
{
  DisplayIntensity();
  if (DisableDisplay)
  {
    // If serial is enabled then send message
    if (!SerialMute) serial->println("Button pressed.");
    return;
  }
  ButtonPressed = true;
}

// Watchdog timer setup routine, this function will enable the watchdog timer.
// call WDT_Restart(WDT) at least every 2.5 sec to stop an automatic reset.
// the setup routine runs once when you press reset:
#ifdef UseWDT
//double timeout = 5.0;  // number of seconds for timeout
double timeout = 8.0;  // number of seconds for timeout
int timeout2 = 0;
void WDT_Setup () {

#ifdef TestMode
  return;
#endif

  timeout2 = (int)( timeout * 227);

  timeout2 = 0x0fff2000 + timeout2;
  WDT_Enable(WDT, timeout2);

  // number of loops:
  // 0x0fff2000 0
  // 0x0fff200f 231
  // 0x0fff2fff 2981
}
#endif

void Signon(void)
{
  bool PBwasPressed = false;

  // If the controll push button is pressed then wait for it to be released before
  // proceeding.
#ifndef TestMode
  while (digitalRead(PB) == HIGH) PBwasPressed = true;
#endif
  delay(100); // bounce delay
  //
  tft.fillScreen(ILI9340_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9340_YELLOW, ILI9340_BLACK); tft.setTextSize(8);
  tft.println("MIPS");
  tft.setTextSize(1);
  tft.println("  Modular");
  tft.println("  Intelligent");
  tft.println("  Power");
  tft.println("  Sources\n");
  tft.println(Version);
  tft.println("GAA Custom Electronics, LLC\n\n");
  tft.setTextSize(2);
  tft.setTextColor(ILI9340_GREEN, ILI9340_BLACK);
  tft.println("\nSystem initialized!");
  if (MIPSconfigData.StartupHold)
  {
    tft.println("\nPress knob to load");
    tft.println("parameters and startup.");
  }
  else
  {
    tft.println("\nPress knob to startup");
    if (PBwasPressed) tft.println("using default parameters.");
  }
  ButtonPressed = false;
  int i = 0;
  while (true)
  {
    if (i >= MIPSconfigData.StartupDelay) break;
    if (ButtonPressed)
    {
      if (!MIPSconfigData.StartupHold) if (PBwasPressed) NormalStartup = false; // if false don't load from EEPROM on cards, use defaults!
      break;
    }
    PB_GREEN_ON;
    delay(500);
    WDT_Restart(WDT);
    PB_GREEN_OFF;
    delay(500);
    WDT_Restart(WDT);
    if (!MIPSconfigData.StartupHold) i++;
  }
  PB_GREEN_OFF;
}

// This function is called every 500 milli sec and its designed to drive the
// LEDs in the push button. It processes requests defined in:
// int    PBled;
// PBledStates  PBledMode;
void ProcessLED()
{
  static int i = 0;

  i ^= 1;   // Toggle i between 0 and 1
  if (LEDoverride)
  {
    if ((i == 1) && ((LEDstate & 8) != 0))
    {
      // Blinking and this is off time!
      PB_RED_OFF;
      PB_GREEN_OFF;
      PB_BLUE_OFF;
      return;
    }
    // Here if the LED override flag is set
    if ((LEDstate & 1) != 0) PB_RED_ON;
    else PB_RED_OFF;
    if ((LEDstate & 2) != 0) PB_BLUE_ON;
    else PB_BLUE_OFF;
    if ((LEDstate & 4) != 0) PB_GREEN_ON;
    else PB_GREEN_OFF;
    return;
  }
  // If in suspend mode let LED glow orange
  if (Suspend)
  {
    PB_RED_ON;
    PB_GREEN_ON;
    PB_BLUE_OFF;
    return;
  }
  if (PBledMode == NOTHING) return;
  if (PBledMode == ON)
  {
    PB_RED_OFF;
    PB_GREEN_OFF;
    PB_BLUE_OFF;
    digitalWrite(PBled, LOW);
  }
  if (PBledMode == OFF)
  {
    PB_RED_OFF;
    PB_GREEN_OFF;
    PB_BLUE_OFF;
    digitalWrite(PBled, HIGH);
  }
  if (PBledMode == FLASH)
  {
    if (digitalRead(PBled) == LOW)
    {
      PB_RED_OFF;
      PB_GREEN_OFF;
      PB_BLUE_OFF;
      digitalWrite(PBled, HIGH);
    }
    else digitalWrite(PBled, LOW);
  }
}

// I think this will intercept the system tic interrupt, test it!
// Tested but it is not getting called!
int sysTickHook(void)
{
  //  LDAChigh;
  //  LDAClow;
  //  return(0);
}

// this one was getting called until the init function ended...
void yield(void)
{
  //  LDAChigh;
  //  LDAClow;
}

//int sysTickHook(void) __attribute__ ((weak, alias("__mysysTickHook")));

void testISR()
{
  //   AtomicBlock< Atomic_RestoreState > a_Block;
  //   NVIC_SetPriority(TC1_IRQn, 15);
  Timer1.setPriority(15);
  for (int i = 0; i < 100; i++) delayMicroseconds(100);
}

void test(void)
{
  Timer1.attachInterrupt(testISR);
  Timer1.start(100000); // Calls every 100ms
}

void setup()
{
  //  int i = REG_RSTC_SR;  // Reads the boot flag
  analogReadResolution(12);
  Reset_IOpins();
  ClearDOshiftRegs();
  SPI.setClockDivider(21);
  pinMode(_sdcs, OUTPUT);
  digitalWrite(_sdcs, HIGH);
  delay(100);
  // Init the display and setup all the hardware
  pinMode(BACKLIGHT, INPUT_PULLUP);
  SetBackLight();
  // Pin 49 defines the display type, if grounded is the HX8347_DSP else ILI9340_DSP
  pinMode(49,INPUT_PULLUP);
  tft.SetDisplayType(ILI9340_DSP); 
  tft.begin();
  tft.fillScreen(ILI9340_BLACK);
  tft.setRotation(1);
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9340_WHITE, ILI9340_BLACK);
  tft.setTextSize(1);
  tft.println("Initializing....");

  SerialInit();
  delay(250);

  SPI.setClockDivider(21);
  SPI.setClockDivider(30);
  pinMode(_sdcs, OUTPUT);
  digitalWrite(_sdcs, HIGH);

  digitalWrite(_sdcs, LOW);
  delay(100);
  digitalWrite(_sdcs, HIGH);
//  delay(100);
//  SPI.setDataMode(_sdcs, SPI_MODE1);
//  SPI.transfer(_sdcs, 0, SPI_CONTINUE);  // Generate some clock pulses
//  SPI.transfer(_sdcs, 0);

  delay(100);
  SD.begin(_sdcs);
  WDT_Restart(WDT);
  delay(100);
  if (!SD.begin(_sdcs))
  {
    tft.println("SD disk failed to initalize!");
    delay(1000);
    SDcardPresent = false;
  }
  else
  {
    SDcardPresent = true;
    //LOADparms("default.cfg"); // Load the defaults if the file is present
  }
  LOADparms("default.cfg");   // Load the defaults if the file is present
  SPI.setClockDivider(11);    // If SD card init fails it will slow down the SPI interface
  SetBackLight();
  DisplayIntensity();
  MIPSsystemLoop();
  if (bmpDraw(MIPSconfigData.BootImage, 0, 0))
  {
    DisableDisplay = true;
    tft.disableDisplay(DisableDisplay);
  }
  else MIPSconfigData.BootImage[0] = 0;
  // Setup the hardware pin directions. Note setting pinMode to output
  // will drive the output pin high.
  Init_IOpins();
  // Clear the digital IO
  ClearDOshiftRegs();
  PulseLDAC;
  // Startup the encoder
  enc.start(PB_PHASE_A, PB_PHASE_B, PB);
  enc.setValue(&encValue);
  enc.attachInterruptChange(encChanged);
  enc.attachInterruptPushButton(encPB);
  // Start the SPI interface
  SPI.begin(SPI_CS);
  SPI.setClockDivider(SPI_CS, 8);  // Default divider is 21 and equals 4 MHz
  // Increase speed for better performance of tables
  // Start the TWI interface
  TWI_RESET();

  Wire.begin();
  Wire.setClock(100000);
  //  Timer3.attachInterrupt(RealTimeISR);
  //  Timer3.start(100000); // Calls every 100ms
  // Initial splash screen
  Signon();
  ButtonPressed = false;
  ButtonRotated = false;
  // Init the baords
  ScanHardware();
  DIO_init();
  // Configure Threads
  MIPSsystemThread.setName("System");
  MIPSsystemThread.onRun(MIPSsystemLoop);
  MIPSsystemThread.setInterval(10);
  LEDThread.setName("LED");
  LEDThread.onRun(ProcessLED);
  LEDThread.setInterval(500);
  // Add threads to the controller
  control.add(&MIPSsystemThread);
  control.add(&LEDThread);
  // If a startup masco is defined, play it now
  if (strlen(MIPSconfigData.StartupMacro) > 0)
  {
    MacroPlay(MIPSconfigData.StartupMacro, true);
  }
  Ethernet_init();
}

// This task is called every 10 milliseconds and handels a number of tasks.
void MIPSsystemLoop(void)
{
  if((BrightTime + 30000) < millis()) SetBackLight();
  float MaxVoltage;
  // Process any serial commands
  // Not sure why this is here??
  //  while(ProcessCommand()==0);  // Process until flag that there is nothing to do, removed Dec 2, 2017
  // Determine the maximum output voltage and set the proper button color
  MaxVoltage = MaxRFVoltage;
  if (MaxDCbiasVoltage > MaxVoltage) MaxVoltage = MaxDCbiasVoltage;
  if (MaxTwaveVoltage > MaxVoltage) MaxVoltage = MaxTwaveVoltage;
  if (MaxFAIMSVoltage > MaxVoltage) MaxVoltage = MaxFAIMSVoltage;
  if (MaxESIvoltage > MaxVoltage) MaxVoltage = MaxESIvoltage;
  // Test output voltages and update the button LEDs to provide warning to user
  // Blue LED on if voltgaes are enabled and above 0
  if (MaxVoltage == 0) {
    PB_OFF(PB_RED);
  }
  if (MaxVoltage >  0) {
    PB_ON(PB_BLUE);
  }
  if (MaxVoltage > 50) {
    PB_ON(PB_RED);
  }
  if (MaxVoltage > 75) {
    PB_FLASH(PB_RED);
  }
  // Test Vin, if power is off then shut down the hardware control lines and print
  // and warding on the display. Process serial commands and stay in this loop until
  // power is reapplied. Reset when power is reapplied
#ifdef TestMode
  return;
#endif
  if (!AcquireADC()) return;
  if (ReadVin() < 10.0)
  {
    tft.disableDisplay(false);
    // Set all control lines to input, this will keep the systems from sourcing power to
    // the modules through driven outputs.
    FilamentShutdown();
    ClearDOshiftRegs();
    Reset_IOpins();
    // Display a message on the screen that MIPS power is off.
    tft.fillScreen(ILI9340_BLACK);
    tft.setRotation(1);
    if (strlen(MIPSconfigData.BootImage) <= 0)
    {
      tft.setCursor(0, 0);
      tft.setTextColor(ILI9340_WHITE, ILI9340_BLACK);
      tft.setTextSize(2);
      tft.println("");
      tft.println("MIPS power is off!");
      tft.println("USB powering controller!");
      tft.println("Apply power to operate...");
    }
    // Dim the display, we could be like this for a long time!
    if(MIPSconfigData.EnetUseTWI) analogWrite(BACKLIGHT, 0);
    else analogWrite(BACKLIGHT, 400);
    //    bmpDraw(MIPSconfigData.BootImage, 0, 0);
    // Wait for power to appear and then reset the system.
    while (ReadVin() < 10.0) WDT_Restart(WDT);
    ReleaseADC();
    Software_Reset();
  }
  ReleaseADC();
}

// This function will scan through all the posible addresses for the EEPROMs that are loacted on
// the IO cards. If a EEPROM is found its read to determine the type of card. The proper init
// function is then called. If the signature is not valid then nothing is done.
// The following are valid addresses for the EEPROM:
// 0x50
// 0x52
// 0x54
// 0x56
// This same set of addresses appears for board select A and B
void ScanHardware(void)
{
  uint8_t addr;
  char    signature[100];

#ifdef TestFilament
  Filament_init(0);
#endif
#ifdef TestTwave
  Twave_init(0, 0x52);
  Twave_init(1, 0x52);
#endif
  // Loop through all the addresses looking for signatures
  for (addr = 0x50; addr <= 0x56; addr  += 2)
  {
    // Set board select to A
    ENA_BRD_A;
    if (ReadEEPROM(signature, addr, 0, 100) == 0)
    {
      // Test the signature and init the system if the signature is known
      if (strcmp(&signature[2], "DAC") == 0) DAC_init(0, addr);
      if (strcmp(&signature[2], "Twave") == 0) Twave_init(0, addr);
      if (strcmp(&signature[2], "RFdriver") == 0) RFdriver_init(0, addr);
      if (strcmp(&signature[2], "DCbias") == 0) DCbias_init(0, addr);
      if (strcmp(&signature[2], "ESI") == 0) ESI_init(0, addr);
      if (strcmp(&signature[2], "Filament") == 0) Filament_init(0, addr);
      if (strcmp(&signature[2], "ARB") == 0) ARB_init(0, addr);
      if (strcmp(&signature[2], "HOFAIMS") == 0) HOFAIMS_init(0, addr);
      if (strcmp(&signature[2], "RFamp") == 0) RFA_init(0, addr);
    }
    // Set board select to B
    ENA_BRD_B;
    if (ReadEEPROM(signature, addr, 0, 100) == 0)
    {
      // Test the signature and init the system if the signature is known
      if (strcmp(&signature[2], "DAC") == 0) DAC_init(1, addr);
      if (strcmp(&signature[2], "Twave") == 0) Twave_init(1, addr);
      if (strcmp(&signature[2], "RFdriver") == 0) RFdriver_init(1, addr);
      if (strcmp(&signature[2], "DCbias") == 0) DCbias_init(1, addr);
      if (strcmp(&signature[2], "ESI") == 0) ESI_init(1, addr);
      if (strcmp(&signature[2], "Filament") == 0) Filament_init(1, addr);
      if (strcmp(&signature[2], "ARB") == 0) ARB_init(1, addr);
      if (strcmp(&signature[2], "HOFAIMS") == 0) HOFAIMS_init(1, addr);
      if (strcmp(&signature[2], "RFamp") == 0) RFA_init(1, addr);
    }
  }
  // Now look for the FAIMS board. This is done last because field driven FAIMS mode is selected if
  // a DCbias module is found.
  for (addr = 0x50; addr <= 0x56; addr  += 2)
  {
    // Set board select to A
    ENA_BRD_A;
    if (ReadEEPROM(signature, addr, 0, 100) == 0)
    {
      // Test the signature and init the system if the signature is known
      if (strcmp(&signature[2], "FAIMS") == 0) FAIMS_init(0);
    }
    // Set board select to B
    ENA_BRD_B;
    if (ReadEEPROM(signature, addr, 0, 100) == 0)
    {
      // Test the signature and init the system if the signature is known
      if (strcmp(&signature[2], "FAIMS") == 0) FAIMS_init(1);
    }
  }
  WiFi_init();
  // The last thig we do is look for the analog input option
  if (MIPSconfigData.UseAnalog) Analog_init();
}

// This function allows using a host computer to navigate the MIPS UI,
// This feature has to first be enabled using a host command to enable.
bool SerialNavigation(char c)
{
  if (!EnableSerialNavigation) return false;
  switch ((int)c)
  {
    case 9:
      enc.SetPB();
      return true;
    case 28:
      enc.SetChange(1);
      enc.SetRotate();
      return true;
    case 29:
      enc.SetChange(10);
      enc.SetRotate();
      return true;
    case 30:
      enc.SetChange(-1);
      enc.SetRotate();
      return true;
    case 31:
      enc.SetChange(-10);
      enc.SetRotate();
      return true;
    default:
      return false;
      break;
  }
  return false;
}

void ReadAllSerial(void)
{
  WDT_Restart(WDT);
  // Put serial received characters in the input ring buffer
  while (SerialUSB.available() > 0)
  {
    ResetFilamentSerialWD();
    serial = &SerialUSB;
    char c = SerialUSB.read();
    if (!SerialNavigation(c)) PutCh(c);
  }
#ifdef EnableSerial
  if ((!MIPSconfigData.UseWiFi) || (wifidata.SerialPort != 0))
  {
    while (Serial.available() > 0)
    {
      ResetFilamentSerialWD();
      serial = &Serial;
      PutCh(Serial.read());
    }
  }
#endif
  while (WiFiSerial->available() > 0)
  {
    ResetFilamentSerialWD();
    serial = WiFiSerial;
    PutCh(WiFiSerial->read());
  }
}

// This function process all the serial IO and commands
void ProcessSerial(void)
{
  // Put serial received characters in the input ring buffer
  while (SerialUSB.available() > 0)
  {
    ResetFilamentSerialWD();
    serial = &SerialUSB;
    char c = SerialUSB.read();
    if(Serial1Echo)
    {
       Serial1.write(c);    // Serial1 is also WiFiSerial
       Serial1.flush();
    }
    if (!SerialNavigation(c)) PutCh(c);
  }
#ifdef EnableSerial
  if ((!MIPSconfigData.UseWiFi) || (wifidata.SerialPort != 0))
  {
    while (Serial.available() > 0)
    {
      ResetFilamentSerialWD();
      serial = &Serial;
      PutCh(Serial.read());
    }
  }
#endif
  while (WiFiSerial->available() > 0)
  {
    ResetFilamentSerialWD();
    serial = WiFiSerial;
    PutCh(WiFiSerial->read());
  }
  ProcessEthernet();
  // If there is a command in the input ring buffer, process it!
  while (RB_Commands(&RB) > 0) // Process until flag that there is nothing to do
  {
    while (ProcessCommand() == 0) WDT_Restart(WDT);
  }
  SerialUSB.flush();     // Added 9/2/2017
  DIOopsReport();
}

// Main processing loop
void loop()
{
  static bool DisableDisplayStatus = false;

  if ((!DisableDisplay) && (DisableDisplayStatus))
  {
    // Here is the display was disabled and it now going to be enabled.
    // Clear the display and reprint the current menu or dialog.
    tft.disableDisplay(DisableDisplay);
    tft.fillScreen(ILI9340_BLACK);
    if (ActiveMenu != NULL) MenuDisplay(ActiveMenu);
    if (ActiveDialog != NULL) DialogBoxDisplay(ActiveDialog);
  }
  DisableDisplayStatus = DisableDisplay;
  tft.disableDisplay(DisableDisplay);
  WDT_Restart(WDT);
  // run ThreadController
  // this will check every thread inside ThreadController,
  // if it should run. If yes, he will run it;
  if (!Suspend) control.run();
  else  ProcessLED();
  // Process any encoder event
  if (ButtonRotated)
  {
    ButtonRotated = false;
    if (ActiveMenu != NULL) MenuProcessChange(ActiveMenu, encValue);
    else if (ActiveDialog != NULL) DialogBoxProcessChange(ActiveDialog, encValue);
    encValue = 0;
  }
  if (ButtonPressed)
  {
    Suspend = false;
    delay(10);
    ButtonPressed = false;
    encValue = 0;
    if (ActiveMenu != NULL) MenuButtonPress(ActiveMenu);
    else if (ActiveDialog != NULL) DialogButtonPress(ActiveDialog);
    DismissMessageIfButton();
  }
  ProcessSerial();
  // Interlock processing.
  // If the user has defined an interlock input then monitor this channel.
  // Arm when it goes high, if armed and it goes lown trip the power supplies.
  // If the user has defined an interlock output then drive the signal high
  // when the power supplies are on.
  static bool initInterlock = true;
  static int  WasPowerON = digitalRead(PWR_ON);  // LOW = on
  static bool InterlockArmed = false;
  if((MIPSconfigData.InterlockIn >= 'Q') && (MIPSconfigData.InterlockIn <= 'X'))
  {
    if(InterlockArmed)
    {
      if(ReadInput(MIPSconfigData.InterlockIn) == LOW)
      {
        // Trip power supply and disarm interlock
        MIPSconfigData.PowerEnable = false;
        digitalWrite(PWR_ON,HIGH);
        DisplayMessageButtonDismiss("Interlock Trip!");
        InterlockArmed = false;
      }
    }
    else if(ReadInput(MIPSconfigData.InterlockIn) == HIGH) InterlockArmed = true;
  }
  if((initInterlock) || (WasPowerON != digitalRead(PWR_ON)))
  {
    initInterlock = false;
    if((MIPSconfigData.InterlockOut >= 'A') && (MIPSconfigData.InterlockOut <= 'P'))
    {
      // If power supply is on then output interlock high 
      if(NumberOfDCChannels > 0)
      {
        WasPowerON = digitalRead(PWR_ON);
        if(digitalRead(PWR_ON) == LOW) SetOutput(MIPSconfigData.InterlockOut,HIGH);
        else SetOutput(MIPSconfigData.InterlockOut,LOW);
      }
      else SetOutput(MIPSconfigData.InterlockOut,HIGH);
      UpdateDigitialOutputArray();
    }
  }
}


// Host command processing function.

// Save parameters to default.cfg file on SD card
int SAVEparms(char *filename)
{
  File file;

  MIPSconfigData.DisableDisplay = DisableDisplay;
  MIPSconfigData.signature = 0xA55AE99E;
  {
     AtomicBlock< Atomic_RestoreState > a_Block;
     // Save to FLASH as well in case SD card fails or is not present
     dueFlashStorage.writeAbs((uint32_t)NonVolStorage, (byte *)&MIPSconfigData, sizeof(MIPSconfigStruct));
  }
  // Test SD present flag, exit and NAK if no card or it failed to init
  if (!SDcardPresent) return (ERR_NOSDCARD);
  SD.begin(_sdcs);
  // Remove the existing default.cfg file
  SD.remove(filename);
  // Open file and write config structure to disk
  if (!(file = SD.open(filename, FILE_WRITE))) return (ERR_CANTCREATEFILE);
  MIPSconfigData.Size = sizeof(MIPSconfigStruct);
  file.write((byte *)&MIPSconfigData, sizeof(MIPSconfigStruct));
  file.close();
  return (0);
}

void SAVEparms(void)
{
  int iStat;

  if ((iStat = SAVEparms("default.cfg")) == 0)
  {
    SendACK;
    return;
  }
  SetErrorCode(iStat);
  SendNAK;
  return;
}

// This function load the MIPS config structure from the filename passed.
// The size of the struct on disk is used, this allows the struct to grow
// and new additional (at the bottom of the struct) will use the default
// values.
// Returns true if all went well!
bool LOADparms(char *filename)
{
  File file;
  MIPSconfigStruct MCS;
  int  i, fVal;
  byte *b;

  while(1)
  {
    // Test SD present flag
    if (!SDcardPresent) break;
    SD.begin(_sdcs);
    // Open the file
    if (!(file = SD.open(filename, FILE_READ))) break;
    // read the data
    b = (byte *)&MCS;
    for (i = 0; i < sizeof(MIPSconfigStruct); i++)
    {
      if ((fVal = file.read()) == -1) break;
      b[i] = fVal;
    }
    file.close();
    // Copy to MIPS config struct
    if(MCS.signature == 0xA55AE99E)
    {
       memcpy(&MIPSconfigData, &MCS, MCS.Size);
       DisableDisplay = MIPSconfigData.DisableDisplay;
       return true;
    }
    break;
  }
  // If here then the SD card read failed so try loading from FLASH
  b = (byte *)&MCS;
  for(i=0;i<sizeof(MIPSconfigStruct);i++)
  {
    b[i] = dueFlashStorage.readAbs(((uint32_t)NonVolStorage) + i);  
  }
  // Check signature, if correct then update
  if(MCS.signature == 0xA55AE99E)
  {
    memcpy(&MIPSconfigData, &MCS, MCS.Size);
    DisableDisplay = MIPSconfigData.DisableDisplay;
    return true;
  }
  return false;
}



