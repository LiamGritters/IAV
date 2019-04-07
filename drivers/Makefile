#### Makefile for IAV Device Drivers ### 

############### MAKE ###############

all:	
	$(MAKE) -C ./ArduinoLinearActuatorDriver
	$(MAKE) -C ./SevconGen4MotorController
	$(MAKE) -C ./JoystickController

	#test component/utilities
	$(MAKE) -C ./ArduinoLinearActuatorDriver/comp
	$(MAKE) -C ./SevconGen4MotorController/comp
	$(MAKE) -C ./VectorNavVN300/comp

	
############### CLEAN ###############

.PHONY: clean

clean:
	$(MAKE) clean -C ./ArduinoLinearActuatorDriver
	$(MAKE) clean -C ./SevconGen4MotorController
	$(MAKE) clean -C ./JoystickController

    #test component/utilities 
	$(MAKE) clean -C ./ArduinoLinearActuatorDriver/comp
	$(MAKE) clean -C ./SevconGen4MotorController/comp
	$(MAKE) clean -C ./VectorNavVN300/comp