## Microcontroller

### Hardware

The microcontroller code targets the STM32F466RE microcontroller.

### Features

#### Pedal Controller

Detects faults so the microcontroller can turn off torque vectoring if something is wrong.

#### Frequency Measurement

Measures frequency from front wheels to find the car's velocity. Discards outliers in the incoming frequency.

#### SD Card Logging

Provides functions for logging information to an SD card.

#### Communications

Functions for interfacing with the ADC, DAC, and with CAN.

#### Torque Vectoring Calculations

Functions for converting between different forms of data.

Calculate the e-diff using the lookup table generated from the python simulation.

#### Lookup Table

A collection of functions to interface with the lookup table, including interpolation between the discrete values.

