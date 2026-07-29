Main.cpp functionality. 

Ashton: 
Main(): 
- initialises the Pico pins and ADC (essentially, everything necessary) 

activate_buzzer() 
When triggered, sets the buzzer to ring for an amount of time specified when calling the function. 

detect_car()
causes for the system to detect when a car has passed through the breakbeam sensor. 
Once the light is broken, it calls the activate buzzer to audibly notify people watching.
Once the sensor is now picking up that the light is unbroken again, it assumes it was interrupted by a car and stores timer and lap information
