# PIC18F46Q71 X2 Timer + Trigger Box

## Introduction

The X2-timer+trigger box is built around a PIC18F46Q71-I/P microcontroller, 
which monitors one or two analog input channels (INa and INb) and produces
step changes on 8 digital output channels (OUT0 .. OUT7) at specific times.
The times for the output transitions are determined from events on
the input channels and the mode of operation.

Because the microcontroller is powered via a USB cable (TTL-232-5V from FTDI), 
the system is essentially a 5V system.
The input voltages should be kept within the range 0 to 5V 
and the digital outputs will have a logic high value that is close to 5V.

Operator interaction with the microcontroller (MCU) is via 
simple text commands sent through the same USB cable.
The expected serial port configuration is 115200 baud, 8-bit data, 
no parity and one stop bit, with RTSCTS hardware flow control.
The MCU does not echo the characters sent to it and expects incoming 
lines of text to end with carriage-return (CR) character.
When sending text to the PC, the MCU ends each line with new-line 
(line-feed, LF) character. 
For operator convenience when entering commands manually, 
the terminal program should have local echo enabled and 
also convert LF line endings to CRLF automatically.

## Modes of operation

When describing the actions of the box, we define some *events*:

- EVENT1 is the time at which the INa voltage crosses a set threshold.
- EVENT2 is the time at which the INb voltage crosses a second threshold.
- EVENT3 is a third time, some (computed) time period following EVENT2.

There are two modes of operation:

- Mode 0: Simple trigger on EVENT1 only.
- Mode 1: Time-of-flight trigger at a computed time, 
  following EVENT1 and then EVENT2.

### Simple trigger mode

The output transitions (low to high) occur at the following times:

| OUTPUT |  TIME |
|--------|:------|
| 0      | EVENT1 + delay0 |
| 1      | EVENT1 + delay1 |
| 2      | EVENT1 + delay2 |
| 3      | EVENT1 |
| 4      | EVENT1 |
| 5      | EVENT1 |
| 6      | EVENT1 |
| 7      | EVENT1 |

Note that, if delayN is set to zero,
the corresponding digital output will step high at EVENTa. 

### Time-of-flight (TOF) trigger mode

The output transitions (low to high) occur at the following times:

| OUTPUT |  TIME |
|--------|:------|
| 0      | EVENT3 + delay0 |
| 1      | EVENT3 + delay1 |
| 2      | EVENT3 |
| 3      | EVENT3 |
| 4      | EVENT3 |
| 5      | EVENT3 |
| 6      | EVENT2 |
| 7      | EVENT1 |

## Commands and Configuration

The form of each command is a single character,
possibly followed by numerical data (integers only).

| Command     |  Comments/Action |
|-------------|:-----------------|
| `h` or `?`  | print the help message |
| `v`         | report version of firmware |
| `n`         | report number of registers |
| `p`         | report register values |
| `r <i>`     | report value of register `i` |
| `s <i> <j>` | set register `i` to value `j` |
| `R`         | restore register values from EEPROM |
| `S`         | save register values to EEPROM |
| `F`         | set register values to original values |
| `a`         | arm device and wait for event |
| `c <i>`     | convert analogue channel `i` (12-bit result, 0-4095) |
|             | i=57 to read threshold voltage on INa (DAC2_output) |
|             | i=58 to read threshold voltage on INb (DAC3_output) |
|             | i=0  to read incoming voltage on INa (RA0/C1IN0-) |
|             | i=9  to read incoming voltage on INb (RB1/C2IN3-) |

Note that box powers up with a focus on the serial port and will
respond to commands.
Once the box is *armed*, the MCU will focus only on the analog
input signal(s) and block/ignore all serial port communication.
Either a trigger event occurs or you have to press the RESET button
to regain its attention.

The run-time configuration is determined by the values in the (virtual) registers. 

| Register | Parameter               | Default | Comment |
|----------|:------------------------|:--------|:--------|
| 0        | mode                    | 0       | 0= simple trigger from INa signal |
|          |                         |         | 1= time-of-flight(TOF) trigger |
| 1        | threshold level for INa | 5       | an 8-bit number 0-255 |
| 2        | threshold level for INb | 5       | an 8-bit number 0-255 |
| 3        | Vref selection for DACs | 3       | 0=off, 1=1v024, 2=2v048, 3=4v096 |
| 4        | delay0                  | 0       | a 16-bit count (8 ticks per us) |
| 5        | delay1                  | 0       | a 16-bit count (8 ticks per us) |
| 6        | delay2                  | 0       | a 16-bit count (8 ticks per us) |


