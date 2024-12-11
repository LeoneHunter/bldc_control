# bldc_control
This project uses a stm32f4 microcontroller to drive a quadcopter bldc motor.
The microcontroller drives a 12V transistor bridge to generate a three phase voltage for the motor.
A motor control is implemented using "back-EMF Sliding-Mode Observer" which extracts a motor rotor position from it's induced emf.
