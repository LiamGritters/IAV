

const byte numLEDs = 2;
byte ledPin[numLEDs] = {12, 13};
unsigned long LEDinterval[numLEDs] = {200, 400};
unsigned long prevLEDmillis[numLEDs] = {0, 0};

const byte buffSize = 40;
char inputBuffer[buffSize];
const char startMarker = '<';
const char endMarker = '>';
byte bytesRecvd = 0;
boolean readInProgress = false;
boolean newDataFromPC = false;

char messageFromPC[buffSize] = {0};
int TargetPOS = 0;

unsigned long curMillis;

unsigned long prevReplyToPCmillis = 0;
unsigned long replyToPCinterval = 1000;

long CurrentPOS = 0;

const int Accuracy = 20; 

const float LinearActuatorZeroPosition = 20.0; // [cm]
const float LinearActuatorExtended = 25.4; //[cm]
const float LinearActuatorRetracted = 5.5; //[cm]
const float PotentiometerExtended = 930; // reading from potentiometer
const float PotentiometerRetracted = 30; // reading from potentiometer

//initializes the sensor, output,intput pins
int POT = A0; //actuator pot

#define PWM 9 //8 //output pwm
#define DIR 8 //9 //output dir

//=============

void setup() {
  Serial.begin(9600);

    /// defines the PWM and directtion pins as outputs
  pinMode(PWM, OUTPUT);
  pinMode(DIR, OUTPUT);
  
    // flash LEDs so we know we are alive
  for (byte n = 0; n < numLEDs; n++) {
     pinMode(ledPin[n], OUTPUT);
     digitalWrite(ledPin[n], HIGH);
  }
  delay(500); // delay() is OK in setup as it only happens once
  
  for (byte n = 0; n < numLEDs; n++) {
     digitalWrite(ledPin[n], LOW);
  }
  
    // tell the PC we are ready
  Serial.println("<Arduino is ready>");
}

//=============

void loop() {
  curMillis = millis();
//  CurrentPOS = analogRead(POT);// read the analog sensor

  getDataFromPC();

  moveActuator(TargetPOS);

  CurrentPOS = analogRead(POT);// read the analog sensor
  
  replyToPC();
}

//=============

void getDataFromPC() {

    // receive data from PC and save it into inputBuffer
    
  if(Serial.available() > 0) {

    char x = Serial.read();

      // the order of these IF clauses is significant
      
    if (x == endMarker) {
      readInProgress = false;
      newDataFromPC = true;
      inputBuffer[bytesRecvd] = 0;
      parseData();
    }
    
    if(readInProgress) {
      inputBuffer[bytesRecvd] = x;
      bytesRecvd ++;
      if (bytesRecvd == buffSize) {
        bytesRecvd = buffSize - 1;
      }
    }

    if (x == startMarker) { 
      bytesRecvd = 0; 
      readInProgress = true;
    }
  }
}

//=============
 
void parseData() {

    // split the data into its parts
    
  char * strtokIndx; // this is used by strtok() as an index
  
  strtokIndx = strtok(inputBuffer,",");      // get the first part - the string
  strcpy(messageFromPC, strtokIndx); // copy it to messageFromPC
  
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  const int targetUncorrectedPosition = atoi(strtokIndx);     // convert this part to an integer
  const int PosInCM = targetUncorrectedPosition + LinearActuatorZeroPosition; 
  TargetPOS = (((PosInCM - LinearActuatorExtended) * (PotentiometerRetracted - PotentiometerExtended)) / (LinearActuatorRetracted - LinearActuatorExtended)) + PotentiometerExtended;
}

//=============

void replyToPC() {

  if (newDataFromPC) {
    newDataFromPC = false;
    Serial.print("<Msg ");
    Serial.print(messageFromPC);
    Serial.print(" target ");
    Serial.print(TargetPOS);
    Serial.print(" position:");
    Serial.print(CurrentPOS);
    Serial.print(" Time ");
    Serial.print(curMillis >> 9); // divide by 512 is approx = half-seconds
    Serial.println(">");

    delay(100);
  }
}

void moveActuator(int TargetPosition)
{  
  if (newDataFromPC) 
  {
    if(abs(TargetPosition - CurrentPOS) > Accuracy) 
    {
      if(TargetPosition > CurrentPOS)
      {
        digitalWrite(DIR, LOW); //extends
        digitalWrite(PWM,HIGH);
        delay(100);
      }
      else
      {
        digitalWrite(DIR, HIGH); //retracts
        digitalWrite(PWM,HIGH);  
        delay(100);
      }
    }   
    else //At setpoint 
    {
      digitalWrite(PWM,LOW);  
    }  
  }
}


//=============
