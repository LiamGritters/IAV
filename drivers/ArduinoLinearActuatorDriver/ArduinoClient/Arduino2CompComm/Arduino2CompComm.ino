

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
bool newTargetFlag = false; 

unsigned long curMillis;

unsigned long prevReplyToPCmillis = 0;
unsigned long replyToPCinterval = 1000;

long CurrentPOS = 0;

const int Accuracy = 10; 

const float LinearActuatorZeroPosition = 0.20; // [m]
const float LinearActuatorExtended = 0.25; //[m]
const float LinearActuatorRetracted = 0.068; //[m]
const float PotentiometerExtended = 930; // reading from potentiometer
const float PotentiometerRetracted = 58; // reading from potentiometer

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

  getDataFromPC();

  moveActuator(TargetPOS);
  
  replyToPC();
}

//=============

void getDataFromPC() {

    // receive data from PC and save it into inputBuffer
    
//  if(Serial.available() > 0) {
  while(Serial.available() > 0) {

    char x = Serial.read();

      // the order of these IF clauses is significant
      
    if (x == endMarker) {
      readInProgress = false;
      newDataFromPC = true;
      inputBuffer[bytesRecvd] = 0;
      parseData();
      if(Serial.available() < 12) break;
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
  const float targetUncorrectedPosition = atof(strtokIndx);     // convert this part to an integer
  const float PosInCM = targetUncorrectedPosition + LinearActuatorZeroPosition; 
  const int newTargetPOS = (((PosInCM - LinearActuatorExtended) * (PotentiometerRetracted - PotentiometerExtended)) / (LinearActuatorRetracted - LinearActuatorExtended)) + PotentiometerExtended;

  if(newTargetPOS != TargetPOS) 
  {
    TargetPOS = newTargetPOS; 
    newTargetFlag = true; 
  }
  else
  {
    newTargetFlag = false;   
  }
}

//=============

float convertToMeters(int CurrentPosition) {
  const float uncorrectedPositionMeters = (((CurrentPosition - PotentiometerExtended) * (LinearActuatorRetracted - LinearActuatorExtended)) / (PotentiometerRetracted - PotentiometerExtended)) + LinearActuatorExtended;
  return (uncorrectedPositionMeters - LinearActuatorZeroPosition);
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
    Serial.print(convertToMeters(CurrentPOS));
    Serial.print(" Time ");
    Serial.print(curMillis >> 9); // divide by 512 is approx = half-seconds
    Serial.println(">");

    delay(10);
  }
}

void moveActuator(int TargetPosition)
{  
  if(newDataFromPC && newTargetFlag) 
  {
    unsigned long time = millis();
    CurrentPOS = analogRead(POT);// read the analog sensor
    while(abs(TargetPosition - CurrentPOS) > Accuracy) 
    { 
      if(TargetPosition > CurrentPOS)
      {
        digitalWrite(DIR, LOW); //extends
        digitalWrite(PWM,HIGH);
        delay(10);
      }
      else
      {
        digitalWrite(DIR, HIGH); //retracts
        digitalWrite(PWM,HIGH);
        delay(10);
      }
      CurrentPOS = analogRead(POT);// read the analog sensor

      if( (millis() - time) > 1000 )
      {
        newTargetFlag = true;
        break;
      }
    }
       
    digitalWrite(PWM,LOW);   
    delay(10); 
    CurrentPOS = analogRead(POT);// read the analog sensor
  }
}


//=============
