### Output IMU Data Makefile ###

############### COMPILER ###############

TARGET=DisplayIMU

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
CFLAGS += -I$(IAV_WORKSPACE)/libraries/LCMLibrary/messages/iav_lcm
CFLAGS += -I$(IAV_WORKSPACE)/libraries/LCMLibrary/include
CFLAGS += -I$(IAV_WORKSPACE)/components/SensorManager/include

LDFLAGS=  -llcm -lpthread 

############### LIBRARIES ###############

LIBS  = -L/lib/x86_64-linux-gnu/ 
LIBS += -L/usr/local/lib

############### OBJECTS ###############

DEPS = $(patsubst %.hpp,$(IDIR)/%.d,$(wildcard *.hpp))
OBJ = $(patsubst $(SDIR)/%.cpp, $(BUILD)/%.o, $(wildcard $(SDIR)/*.cpp))

OBJ += $(patsubst $(SEN)/src/%.cpp, $(BUILD)/%.o, $(wildcard $(SEN)/src/*.cpp))

############### BUILD ###############

$(BUILD)/%.o: $(SEN)/src/%.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

$(BUILD)/%.o: $(SDIR)/%.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

$(BIN)/$(TARGET): $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS) $(LDFLAGS)

############### CLEAN ###############

.PHONY: clean

clean:
	rm -r $(BUILD)/* 
	rm -r $(BIN)/*  