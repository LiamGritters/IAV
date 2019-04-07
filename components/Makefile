#### Makefile for IAV Components ### 

############### MAKE ###############

all:	
	$(MAKE) -C ./ManualController
	$(MAKE) -C ./SensorManager/utilities
	$(MAKE) -C ./ControlsManager

	
############### CLEAN ###############

.PHONY: clean

clean:
	$(MAKE) clean -C ./ManualController
	$(MAKE) clean -C ./SensorManager/utilities
	$(MAKE) clean -C ./ControlsManager