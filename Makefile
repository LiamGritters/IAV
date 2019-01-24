#### Makefile for entire IAV Project ### 

############### MAKE ###############

all:	
	$(MAKE) -C ./components
	$(MAKE) -C ./libraries

	
############### CLEAN ###############

.PHONY: clean

clean:
	$(MAKE) clean -C ./components
	$(MAKE) clean -C ./libraries