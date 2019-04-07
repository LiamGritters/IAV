### Joystick Controller Makefile ###

############### INCLUDE ###############

include $(IAV_WORKSPACE)/iav.mk

############### TARGET ###############

TARGET=JoystickController

############### COMPILER ###############

CC=g++

############### DIRECTORIES ###############

SDIR=./src
BIN=./bin
BUILD=./build
IDIR =./include

############### FLAGS ###############

CFLAGS  = -std=c++11 -g -Wall -O2 -I$(IDIR) $(STANDARD_INC)
CFLAGS += -I$(IAV_WORKSPACE)/libraries/LCMLibrary/messages/iav_lcm
CFLAGS += -I$(IAV_WORKSPACE)/libraries/LCMLibrary/include
CFLAGS += -I$(JOYSTICK_LIBRARY_PATH)/cpp/include
CFLAGS += -I/usr/include/libevdev-1.0/

LDFLAGS= -llcm -lpthread -lJoystickLibrary -levdev -ludev

############### LIBRARIES ###############

LIBS  = -L/lib/x86_64-linux-gnu/ 
LIBS += -L/usr/local/lib
LIBS += -L$(JOYSTICK_LIBRARY_PATH)/cpp/build/src

############### OBJECTS ###############

DEPS = $(patsubst %.hpp,$(IDIR)/%.d,$(wildcard *.hpp))
OBJ = $(patsubst $(SDIR)/%.cpp, $(BUILD)/%.o, $(wildcard $(SDIR)/*.cpp))


############### BUILD ###############

$(BUILD)/%.o: $(SDIR)/%.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

$(BIN)/$(TARGET): $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS) $(LDFLAGS)

############### CLEAN ###############

.PHONY: clean

clean:
	rm -rf $(BUILD)/* 
	rm -rf $(BIN)/*  