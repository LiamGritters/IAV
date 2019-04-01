### Directional Controller Makefile ###

############### COMPILER ###############

TARGET=ControlsManager

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
CFLAGS += -I$(IAV_WORKSPACE)/libraries/PIDController/include
CFLAGS += -I$(IAV_WORKSPACE)/drivers/ArduinoLinearActuatorDriver/include
CFLAGS += -I$(IAV_WORKSPACE)/drivers/SevconGen4MotorController/include

LDFLAGS= -llcm -lpthread -lPIDController -lArduinoLinearActuatorDriver -lSevconGen4MotorController

############### LIBRARIES ###############

LIBS  = -L/lib/x86_64-linux-gnu/ 
LIBS += -L/usr/local/lib
LIBS += -L$(IAV_WORKSPACE)/libraries/PIDController/bin
LIBS += -L$(IAV_WORKSPACE)/drivers/ArduinoLinearActuatorDriver/bin
LIBS += -L$(IAV_WORKSPACE)/drivers/SevconGen4MotorController/bin

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