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
SEN = /home/liam/workspace/IAV/components/SensorManager/

############### FLAGS ###############

CFLAGS  = -std=c++11 -g -Wall -O2 -I$(IDIR)
CFLAGS += -I/home/liam/workspace/IAV/libraries/LCMLibrary/messages/exlcm
CFLAGS += -I/usr/local -I/usr/include
CFLAGS += -I/home/liam/workspace/IAV/libraries/LCMLibrary/include
CFLAGS += -I/home/liam/workspace/IAV/components/SensorManager/include

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