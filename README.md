# Leviosa_controller_STM32
Software for the controller of the [contactless electrostatic gripper](https://github.com/Fersaar/contactless_electrostatic_gripper). 

## Features
- four independent pid controllers
- live pid tuning
- ADS1115 integration (AD converter)
- Linearizing the analog distance sensors via calibration table
- Communication with [Python-Gui](https://github.com/Fersaar/Leviosa_Python_GUI) via Serial Interface


uC: STM32F103C8T6 (Bluepill) 

Framework: stm32duino 

