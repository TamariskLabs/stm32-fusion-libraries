# STM32 CANopen Library

# Usage

In order to use this library to Implement CANopen on the device you need to:

 - Add a source and header file called "can_open_device_profile.c/h" to your project that is responsible for setting the default Object Ditionary values for the device.  The format for this file is explained in the Device Profile section below.
  - enable the can bus hardware using the cube configurator tool. The proceedure is outlined below in the Hardware Configuration section below.
 - register handler functions to execute (at the rate defined in the can_open_device_profile file) for each of the 4 possible device states.  These should be registered one time at startup, and before calling fusion_can_open_init().  If you do not need to perform any actions during one of the for states, you can simply not register a function for that state.  The 4 functions to call for registering to each of the operational states are:
    - fusion_can_open_register_intializing()
    - fusion_can_open_register_pre_operatinal()
    - fusion_can_open_register_operational()
    - fusion_can_open_stopped()
 - call ```fusion_can_open_init()``` during the initialization phase of the stm32.
 - call ```fusion_can_open_update()``` as quickly as possible in the main loop of the application.
 - call ```fusion_can_open_send_emergency(CANopenERROR error)``` when appropriate.  You can look in the Error Hanling section below for more details.

 # Hardware Configuration

 # Device Profile

 # Error Handling