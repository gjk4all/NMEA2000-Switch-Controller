# NMEA2000-Switch-Controller
A switch controller sending NMEA2000 PGN 127502 - Switch Bank Control messages

The software is written for and tested on a Blue Pill (stm32f103c8t6)\
It is written in STM32CubeIDE (version 1.19)

The device can service 8 switches and send their PGN 127502 commands. The device supports 4 types of switches:
- (mom)On - Neutral - (mom)Off switches
- (mom)On/Off switches (aka pushbutton)
- (mom)On - Off switches
- (mom)On, switch off after set time
Every switch can be configured to control a specific PGN 127502 relay output.

## Schematic
V1.0 [schematic](NMEA2000%20Switch%20Controller%20V1.0.pdf) added to the repository

## Compliance
- Reacts on PGN 59904 - ISO Request messages (for all supported PGN's)
- Sends NMEA2000 PGN 60928 - ISO Address Claim messages to join the bus
- Reacts on NMEA2000 PGN 60928 - ISO Address Claim messages to resolve address conflicts
- Sends NMEA2000 PGN 127502 - Switch Bank Control messages to switch relays
- Reacts to request for PGN 126996 Product Information
- Reacts on request for PGN 126464 PGN List, sends TX list and RX list (2 responces)
- Reacts to PGN 60160 ISO Transport Protocol DT (for PGN 65240)
- Reacts to PGN 60416 ISO Transport Protocol CM (BAM announcement for PGN 65240)
- Reacts to PGN 65240 ISO Commanded Address
- Reacts to PGN 127501 - Binary Switch Bank Status messages, LED's give actual status of remote relays
- Broadcasts NMEA2000 PGN 126993 Heartbeat messages

## Compliance todo (in future release)
- PGN 59392 ISO Acknowledgment (TX/RX) (implemented but no use yet)
- PGN 126998 Configuration Information (TX)

## Electronics
See [schematic](NMEA2000%20Switch%20Controller%20V1.0.pdf)
- Operating voltage: 12V (9-16V through NMEA2000 bus)
- LED's operating voltage: 12V (NMEA2000 bus voltage)

## Links
[Canboat.github.com](https://canboat.github.io/canboat/canboat.html)\
[mgenergysystems.eu](https://docs.mgenergysystems.eu/en/application-notes/Tracking-MG-device-on-NMEA2000-CAN-bus#:~:text=Address%20Claim%20procedure%20(ACL),send%20by%20this%20device%20first.)\
[embeddedflakes.com](https://embeddedflakes.com/network-management-in-sae-j1939/)

## Disclamer
I try my best to make the device as compliance to NMEA2000 as possible bot don't have the means to certify it with NMEA nor to test it on a real NMEA2000 network. Use of this device and the software is at your own risk! (Don't say i didn't warn you).

The purpose of this project is to learn how the NMEA2000 protocol operates and to share my experiences with the world. Suggestions for improvement are welcome.
