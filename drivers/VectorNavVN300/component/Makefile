### VectorNav VN300 Driver Component Makefile ###

############### INCLUDE ###############

include $(IAV_WORKSPACE)/iav.mk

############### TARGET ###############

TARGET=VectorNavVN300DriverComponent

############### COMPILER ###############

CC=g++

############### DIRECTORIES ###############

SDIR=./src
BIN=./bin
BUILD=./build
IDIR =./include

############### FLAGS ###############

CFLAGS  = -std=c++11 -g -Wall -O2 -I$(IDIR) $(STANDARD_INC)
CFLAGS += -I$(IAV_WORKSPACE)/drivers/VectorNavVN300/include
CFLAGS += -I$(VECTORNAV_PATH)/cpp/include
CFLAGS += -I$(IAV_WORKSPACE)/components/SensorManager/include/
LDFLAGS= -lpthread -llibvncxx

############### LIBRARIES ###############

LIBS  = -L/lib/x86_64-linux-gnu/ 
LIBS += -L/usr/local/lib
LIBS += -L$(VECTORNAV_PATH)/cpp/build

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