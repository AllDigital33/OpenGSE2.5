# Open GSE Controller

This Open GSE Controller project was developed to support the Friends of Amateur Rocketry (FAR) bipropellant liquid student kit rocket. It is designed to handle all aspects of loading, pressurization, and launching of a bipropellant liquid rocket, as well as all the data acquisition (DAQ) needed for a static firing. It has gone through 3-4 hardware and software revisions but is still considered a work in progress. We’ve decided to take a snapshot and share our learnings, so student groups can learn, borrow, or reuse some of our ideas and our sloppy code. The plans, designs, and code provided are intended to be used for student learning only and support of amateur rockets and information that is already widely available in the public domain. 

See the GSE Controller Overview PDF for a comprehensive feature and design overview. This repository contains all the code, schematics, PCB designs, and component summaries for the Open GSE Controller, remotes, and all of the supporting modules.



# Disclaimer

The open-source code and hardware designs ("Materials") provided here are related to the development and launching of bipropellant liquid rockets. The authors and contributors of these Materials provide them for educational and informational purposes only.

**WARNING: LAUNCHING LIQUID ROCKETS IS AN INHERENTLY DANGEROUS ACTIVITY.** The use of the Materials involves significant risks, including, but not limited to, serious physical injury, disability, and death. Anyone using, modifying, or distributing these Materials must understand and accept these risks. If you choose to use these Materials, you do so at your own risk.

The authors and contributors of these Materials make no representations or warranties of any kind, express or implied, about the completeness, accuracy, reliability, suitability, or availability with respect to the Materials. Any reliance you place on such information is therefore strictly at your own risk.

In no event will the authors or contributors be liable for any loss or damage including without limitation, indirect or consequential loss or damage, or any loss or damage whatsoever arising from loss of data or profits arising out of, or in connection with, the use of these Materials.

The authors and contributors disclaim all responsibility for any injuries, damages, or losses of any nature whatsoever, whether direct, indirect, punitive, incidental, special, consequential, or any other damages, to persons or property, arising out of or connected with the use, misuse, or inability to use the Materials provided here.

**BY** **USING THESE MATERIALS, YOU ACKNOWLEDGE THAT YOU HAVE READ THIS DISCLAIMER, UNDERSTAND IT, AND AGREE TO BE BOUND BY IT.**



# GSE Solution Overview

Below are the general design principles that drove the development of the controller and remotes:

1. **Minimal design** – maximum features - Simplified and compact design with minimum features to load, pressurize, launch, and static fire a two-tank bipropellant rocket.
2. **Basic components and IDE’s** - Developed using Arduino-based MCU (Teensy 4.1-Arm7), EasyEDA PCB layout, and two options for remote commanding of controller (iPad Apple xCode IDE or Arduino ”analog” version).
3. **Simplified user interface -** Custom remotes provide for simplifying the human UI, adding error controls, and minimizing the human risk related to loading, pressurizing, and launching.
4. **Safety and error systems** - Automated processes in the controller for pressurizing and quick disconnects, over-current and fault protections, error checking, and more.
5. **Open source** - Code and hardware design available for learning



The overall solution consists of a central pad controller, an iPad-based remote, or an "analog" mini-remote to control all of the valves, pressure monitoring, actuators, recovery systems, main valves, and igniter.  A summary of the features includes:

#### GSE Controller

- Control of six pad valves for lox/fuel pressurization, QD activation, and QD line venting.
- Control of rocket lox/fuel vent valves and lox/fuel pressure monitoring
- Rocket recovery (altimeter) power and armed state monitoring
- Main valve actuation control
- Igniter firing and local continuity check
- Full featured four channel DAQ recorder for static fire analysis
- Two spare pressure transducer inputs
- Hazard power safety key and strobe indicator
- Automated process tasks for pressurization and pneumatic quick disconnects (QDs)

#### iPad Touch-based Remote

- Tab-based control screens including:
- Simple Pad Status of all pressures, valves, recovery, igniter, etc.
- Manual valve control screen
- Automated process control screen
- Recovery power/arm screen
- Launch screen with poll go-no-go status, arm controls, main valves, igniter, and abort
- DAQ screen with graph results and file storage
- Configuration with dozens of settings, PT range and zero, and feature activation
- Key switch arming

#### “Analog” Remote

- Simplified analog remote with control switches, 7-segment displays, and critical indicators
- Can do everything the iPad can do, except DAQ activation/review and advanced configuration
- Key switch arming
- Automated process initiation
- Manual valve control
- Pressure monitoring / pressure setting
- Recovery power
- Igniter control / continuity monitoring
- Main valves actuation control
- Abort



# GSE Controller Details

The GSE Controller consists of a custom PCB with a microcontroller mounted inside a single aluminum project box. It has external I/O ports for pad valves, a rocket tether, igniter, main valve control, a radio antenna, and 12/24vdc power input.

#### Technology summary

- Teensy 4.1 Arm7 Microcontroller w/flash memory DAQ storage
- Arduino IDE / Single C++ Sketch
- NiceRF LoRa 433Mhz serial TTY radio
- 24vdc or 12vdc power x 2 (standard and hazard)
- Quad channel high-side automotive drivers for valve control
- ADS1115 four channel 16bit ADC (x2)
- DB15HD connectors for rocket tether
- DB9 connectors for pad valves
- 4 and 5 pin avionics round connectors for power and igniter
- CPU heatsink strapping for thermal management
- Custom PCB developed using EasyEDA and printed through JLCPCB 

#### Controller Technology Choices

**Teensy 4.1:**  The T4.1 with its Arm Cortex-M7 is just insanely fast for an MCU and it is loaded with a lot of versatile GPIO. The T4.1 also has a built-in SD card for easy transfer of data (although unused). And it easily accommodates a 256mb flash chip (Winbond W25N02KVZEIR) for DAQ logging. The fast MCU allows us to sample four channels of data at 200Hz for static fire testing. We compile at a slower clock speed (384 Mhz) for cooler thermal performance. 

**Radios:** We have tried a number of different radios, but we are very happy with our 1 watt LoRa radios on 433 Mhz (FCC amateur radio license required). You can get similar radios in 900 Mhz that don’t require a license. We use the LoRa 6100Pro serial TTL radios from NiceRF in China. The Lora technology is great and is very reliable.

**ADS1115 16bit ADC:** We use two four channel ADCs. Two channels are used to measure power/battery input voltage and four channels are used for measuring pressure transducers. The ADC’s operate over I2C.

**Valve Drivers – ST VNQ660SPTR-E:** These are four channel 8A high-side automotive drivers for solenoid valve switching. They have a great voltage range, provide extremely fast over-current protection (for shorts), good current capacity, and they have built-in EMF flyback protection. 

**Igniter Driver - Rohm BV1HD090FJ-CE2:** This is a single channel 8A high-side driver used for triggering the igniter. It also provides excellent over-current protection, but doesn’t shut-down when firing an igniter. It has a status pin that provides a safe continuity check. 

**5v Switching - STMPS2252MTR:** We use this 1A high-side driver for external 5v switching and sending a 5v pulse to the recovery board to power it on. A driver is used to provide over-current/short protection for the external controls.

**5V Regulator:**  We use two Pololu 5v 5A switched regulators (D24V50F5) to power the Teensy, external PTs, and the Radio. We run a separate regulator for the radio to minimize RFI and RF blowback, as well, provide the radio with full power – the radio really only needs 1A, so 5A is overkill. All of the valve and igniter events are triggered directly off of the 24v power source and do not go through the regulators. 

**Reverse Polarity Protection**: We use a P-Channel Mosfet (IRF9Z24PBF), along with a Zener diode (SZMMSZ5246BT1G) and 10K resistor to provide reverse polarity protection for the PCB.

**Other**: We use Vishay VO1400AEFTR Optoisolated relays to report fuse status for the three external fuses



# iPad Remote Details

The iPad-based remote consists of a standard iPad Mini paired with a custom bluetooth to serial LoRa radio bridge PCB. The primary application is written in iOS Swift using xCode and facilitates all of the controller capabilities to load, pressurize, launch, or static fire a liquid rocket. 

#### Technology summary

- Standard iPad mini (wifi only)
- Custom Bluetooth bridge using an ESP32 connected to a NiceRF LoRa 433Mhz serial radio
- Apple xCode IDE with iOS Swift code
- 7.2v lipo battery and iPad battery

#### Technology Choices

**iPad Mini:**  When we decided to develop the remote using a touch-based tablet we chose an Apple iPad mini, due to the ubiquity and longevity of the Apple product line. It also provides an extremely intuitive UI which was a major design principle. The iOS Swift IDE (in Apple’s xcode) is free and easy to learn, but to fully exploit Apple’s development environment a $100/year developer fee is required. 

**Bluetooth** **Bridge:** Our original designs used a proprietary cable (Redpark) to connect the iPad to the serial TTY LoRa radio, but we found that the cable was sensitive to RF and would occasionally crash the iPad. The Redpark cable also was not Apple sanctioned and their primary solution was to interface to the iPad using Bluetooth. So we built a small Bluetooth “bridge” from an ESP32 using a small Arduino sketch. The ESP32 receives data from the LoRa serial radio and then passes it to the iPad via Bluetooth. This has proven to be extremely reliable. It also gave us the ability to add a physical key switch to the iPad for Arming the controller. 

**Battery**: EMEPOVGY 2S Lipo Battery Airsoft 1200mAh 7.4V 20C 

**Other:** Our first version of this solution used a full-size 9” iPad, but we found that an iPad Mini worked just fine. Using a project box from Polycase (VM-35MMTT ) provides a great one-handed solution. Also the iPad case from Seymac provides good protection in the rugged environment.



# Analog Remote Details

The "analog remote" supports almost all the features and functions as the iPad remote, except advanced configuration/settings and DAQ capabilities. It is a super compact remote that uses three-way toggle switches and physical push buttons to remotely actuate valves, monitor/set pressure, power on recovery systems, and fire igniters. It also uses the same LoRa serial radios for communication.

#### Technology Summary

- Teensy 4.1 Arm7 Microcontroller
- Arduino IDE / Single C++ Sketch
- NiceRF LoRa 433Mhz serial TTY radio
- Two-factor transmit safety switch
- Key switch for arming
- Seven segment LED displays (x4) for LOX/Fuel pressure monitoring and pressure setting
- Ten three position toggle switches to activate valves and processes
- Six illuminated push buttons to activate processes and report status
- Two potentiometers for setting pressure targets
- 7.2v lipo battery w/10 hour life

#### Technology Choices

**General:** The design consideration for the “analog remote” was to create a super compact remote that had almost all the capabilities as an iPad that could be used as a back-up or as a quick and easy way to check out the rocket and GSE during prep. As it turns out, it required a lot more work and consideration than the “digital” iPad version, since it required more physical design and a LOT of I/O. Also, in order to get it to fit in a super compact package it required two custom PCBs stacked on top of each other. We also wanted to get as many of the switches and components soldered directly to a PCB to reduce the number of board jumpers.

**Teensy 4.1:**  In this situation, the T4.1 is likely overkill. The analog remote does not need much CPU power, but the additional GPIO capacity of the T4.1 was an advantage, so we went with this MCU. That said, we had to add an additional 32 GPIO pins using two 16P GPIO expanders (MCP23017). We are also using a flash chip to save local configuration.



**Radios:** For both remotes we are using the same NiceRF LoRa modules, but the controller uses the 1 watt version, while the remotes use a 100mw version (LoRa611pro). This is a serial TTY radio.

**Displays:** In lieu of any type of LCD or touch display, we needed a way to display pressure, set pressures, and communicate critical information like errors, battery state, etc. So we are using .36” seven-segment LCDs (XDMR09A3) driven by a TM1637 chip. 

**Toggle Switches:** (mon) off (mon) are NKK M2018SS1W13

**Illuminated** **push** **buttons:** C&K D6RLRDF1 LFS

**Potentiometers**: PRS11R-415F-S103B1

**Key** **Switch** **for** **arming:** Digikey KO131A102 

**Case:** Hammond 160x125x31 part: 1455P1601 

**Battery:** Ovonic Air 2200mah 35C 2S



# Other

#### Test Sleds

In addition to the main controller and the remotes, this repository has the schematics and PCB designs for some custom test sleds that "emulate" the rocket subsystems and pad valves and interface to the controller through DB9 or DB15 d-sub connectors. 

#### Recovery Baord

The repository also contains the schematic and PCB layouts for the FAR Remote Recovery Board. This custom PCB is used to remotely power on/off and check the arming status from commercial altimeters on the rocket. It utilizes five wires in the DB15 rocket tether and allows the altimeters to be safely powered up or down after loading of propellant (to save battery).





