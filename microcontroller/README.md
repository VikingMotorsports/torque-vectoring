## Microcontroller

### Hardware & Pinouts

The microcontroller code targets the STM32F466RE microcontroller.
ADC1_IN10: Throttle Input 1 : PC0
ADC2_IN11: Throttle Input 2 : PC1
ADC3_IN12: Steering angle sensor : PC2
ADC3_IN13: Extra analog input (Most likely for steering sensor) : PC3
DAC_OUT1: Left rear throttle : PA4
DAC_OUT2: Right rear throttle : PA5
TIM2_CHANNEL1: Front left wheel frequency detector : PA0
TIM5_CHANNEL2: Front right wheel frequency detector : PA1
CAN1: Inputs from CAN capstone (RPM, current, power) : PA11 & PA12
SPI1: Master output to SD card : Needs 3V and Ground input from microcontroller
Nucleo Pin → SD Pin
PA7 → S1
(Won’t be using S0 for data retrieval from SD card)
PB3 → CLK
PB6 → DET (detection if SD card inserted, GPIO input)
https://www.adafruit.com/product/4682 - SD card breakout board
GPIO: APPS output signal pull-up and open-drain (High when ok, low when not ok): PC7
BSPD fail for CAN system: PC8
Brake Pedal GPIO input (0 for go-ahead, 1 for not ok): PA6

### Features

#### Pedal Controller

Detects faults so the microcontroller can turn off torque vectoring if something is wrong. If the pedal controller detects
if it is faulty, it will wait 70ms to turn off to reduce random turn offs.

#### Frequency Measurement

Measures frequency from front wheels to find the car's velocity. Discards outliers in the incoming frequency.

#### SD Card Logging
SD Logging is currently currently using SPI as the preffered communications protocol.

Write data using the following format:
snprintf(file_name, file_length, "<type1> - ... - <typeN>", <value1>, ... , <valueN>);

Provides the following functions as it pertains to SD Logging:
Mount the SD card to begin logging
sd_mount

Unmount SD card to avoid damage to the card or data saved on the card
sd_unmount

Scan SD card for a node
sd_scan

Format SD card from home directory
sd_format

Write to File
sd_write

Read File
sd_read

Check if SD card is saving to the file
sd_check

Create File
file_create

Update file on SD card
file_update

RemoveFile
file_remove

Create directory in SD card
create_directory

Check space in SD card
sd_space_check


#### Communications

Functions for interfacing with the ADC, DAC, and with CAN. 

#### Torque Vectoring Calculations

Functions for converting between different forms of data.

Calculate the e-diff using the lookup table generated from the python simulation.

#### Lookup Table

A collection of functions to interface with the lookup table, including interpolation between the discrete values.

