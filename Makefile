#### Makefile for entire IAV Project ### 

############### MAKE ###############

all:	
	$(MAKE) -C ./libraries
	$(MAKE) -C ./drivers
	$(MAKE) -C ./components

	
############### CLEAN ###############

.PHONY: clean

clean:
	$(MAKE) clean -C ./libraries
	$(MAKE) clean -C ./drivers
	$(MAKE) clean -C ./components