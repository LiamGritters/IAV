### Sevcon Motorcontroller Driver Makefile ###

############### COMPILER ###############

TARGET=SevconDriver

############### COMPILER ###############

CC=g++

############### DIRECTORIES ###############

SDIR=./src
BIN=./bin
BUILD=./build
IDIR =./include
SEN = $(IAV_WORKSPACE)/components/SensorManager

############### FLAGS ###############

CFLAGS  = -std=c++11 -g -Wall -O2 -I$(IDIR) $(STANDARD_INC)
CFLAGS += -I$(IAV_WORKSPACE)/libraries/PIDController/include
CFLAGS += -I$(IAV_WORKSPACE)/drivers/SevconGen4MotorController/include

LDFLAGS= -lpthread -lPIDController

############### LIBRARIES ###############

LIBS  = -L/lib/x86_64-linux-gnu/ 
LIBS += -L/usr/local/lib
LIBS += -L$(IAV_WORKSPACE)/libraries/PIDController/bin

############### OBJECTS ###############

DEPS = $(patsubst %.hpp,$(IDIR)/%.d,$(wildcard *.hpp))
OBJ = $(patsubst $(SDIR)/%.cpp, $(BUILD)/%.o, $(wildcard $(SDIR)/*.cpp))

OBJ += $(patsubst ../src/%.cpp, $(BUILD)/%.o, $(wildcard ../src/*.cpp))

############### BUILD ###############

$(BUILD)/%.o: ../src/%.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

$(BUILD)/%.o: $(SDIR)/%.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

$(BIN)/$(TARGET): $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS) $(LDFLAGS)

############### CLEAN ###############

.PHONY: clean

clean:
	rm -rf $(BUILD)/* 
	rm -rf $(BIN)/*  