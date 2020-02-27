#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <queue.h>
#include <Arduino.h>
#include <Wire.h>
//Definitions
#define LED_BUILTIN 13

#define SevenSegCC1 44
#define SevenSegCC2 46

#define SevenSegA 4
#define SevenSegB 5
#define SevenSegC 6
#define SevenSegD 7
#define SevenSegE 8
#define SevenSegF 9
#define SevenSegG 10
#define SevenSegDP 11

#define DIP0 53
#define DIP1 51
#define DIP2 49
#define DIP3 47
#define DIP4 45
#define DIP5 43
#define DIP6 41
#define DIP7 39


#define STEPPER_PIN_1 32
#define STEPPER_PIN_2 33
#define STEPPER_PIN_3 34
#define STEPPER_PIN_4 35

#define SSDLetterO 0
#define SSDLetterA 10
#define SSDLetterB 11
#define SSDLetterC 12
#define SSDLetterD 13
#define SSDLetterE 14
#define SSDLetterF 15
#define SSDLetterR 16

// Task handle
TaskHandle_t xHandleStepper;
TaskHandle_t xHandleTempHumid;

QueueHandle_t xQueue;
// table with binary representations of hex values 1 - F on 7 segment display
int hexSSD[] = {63,6,91,79,102,109,125,71,127,103,95,128,89,30,121,113, 51};
//Function Declarations 
int degreeToStep(int val);
int getState(int dipSwitchState);
void OneStep(bool dir);
void setLEDS(int num, bool right);
double readSensor(double* temperature);
// Variable Declarations
int tempHumidVal =0;
int step_number =0;
int currentState;// 0-5 , state 4,5 are block states 
int dipState = 0;
int count1;
int offset;
TickType_t oneSec = 500 / portTICK_PERIOD_MS;
TickType_t dipDelay = 500 / portTICK_PERIOD_MS;
// Task Declarations:
void clockTask( void *pvParameters );
void displayTask( void *pvParameters);
void mainController( void *pvParameters);
//void tempHumidController(void *pvParameters);
void stepperController(void *pvParameters);
void dipswitchController(void *pvParameters);
// Struct contains
struct SSDStruct
{
  int leftDigit;
  int rightDigit;
  // Variable stores number of seconds digit will be displayed
  int displayTime;
};
SSDStruct defaultDisp;

// the setup function runs once when you press reset or power the board
void setup() {
     //Initialize I2C Communication
  Wire.begin();
  Serial.begin(9600);
  //Configure HDC1080
  Wire.beginTransmission(0x40);
  Wire.write(0x02);
  Wire.write(0x90);
  Wire.write(0x00);
  Wire.endTransmission();
  // Initialize Variables
  count1 = 42;
  offset = 1;
  defaultDisp.leftDigit = 4;
  defaultDisp.rightDigit = 2;
  defaultDisp.displayTime = 0;

xQueue = xQueueCreate(10,sizeof(SSDStruct *));
  
  pinMode(DIP0, INPUT);
  pinMode(DIP1, INPUT);
  pinMode(DIP2, INPUT);
  pinMode(DIP3, INPUT);
  pinMode(DIP4, INPUT);
  pinMode(DIP5, INPUT);
  pinMode(DIP6, INPUT);
  pinMode(DIP7, INPUT);

  pinMode(SevenSegA, OUTPUT);
  pinMode(SevenSegB, OUTPUT);
  pinMode(SevenSegC, OUTPUT);
  pinMode(SevenSegD, OUTPUT);
  pinMode(SevenSegE, OUTPUT);
  pinMode(SevenSegF, OUTPUT);
  pinMode(SevenSegG, OUTPUT);
  pinMode(SevenSegDP, OUTPUT);

  pinMode(SevenSegCC1, OUTPUT);
  pinMode(SevenSegCC2, OUTPUT);
  
  pinMode(STEPPER_PIN_1, OUTPUT);
  pinMode(STEPPER_PIN_2, OUTPUT);
  pinMode(STEPPER_PIN_3, OUTPUT);
  pinMode(STEPPER_PIN_4, OUTPUT);

  xTaskCreate(
    //Rename Clock to displayTask updateClock
    displayTask
    ,(const char *) "Display Task"
    ,128
    ,NULL
    ,1
    ,NULL);
  // Rename to clockTask
  xTaskCreate(
    clockTask
    , (const char *) "Clock Task"
    , 128
    , NULL
    , 3
    , NULL);
    // Will control  sensor, stepper and dipswitch behavior 
      xTaskCreate(
    mainController
    , (const char *) "mainController"
    , 128
    , NULL
    , 2
    , NULL);
    /* 
    // This task reads Temperature/ Humidity data from sensor
      xTaskCreate(
    tempHumidController
    , (const char *) "Temperature-Humidity Controller"
    , 128
    , NULL
    , 1
    , &xHandleTempHumid);*/

    // This task controlls stepper motor
      xTaskCreate(
    stepperController
    , (const char *) "Stepper Controller"
    , 128
    , NULL
    , 1
    , &xHandleStepper);
    // This task checks dip switch for changes
      xTaskCreate(
    dipswitchController
    , (const char *) "Dip Switch Controller"
    , 128
    , NULL
    , 1
    , NULL);

}

void loop(){
}
/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

//Periodically checks to see if queue is full and updates the time remaining on the 
// Currently displayed 7 seg display values
void clockTask( void *pvParameters ){
  (void) pvParameters;
  bool inOFState = false;
  for(;;){
    if ( defaultDisp.displayTime  > 0){
      defaultDisp.displayTime -= 1;
    }
    vTaskDelay(oneSec);
}
}
void mainController( void *pvParameters){
  (void) pvParameters;
  int tempVal =0 , humidityVal = 0, state = dipState , stepperState = 0;
  bool blockState = false;
  double temperature;
  double humidity;
  for(;;){
      humidity = readSensor(&temperature);
    if(state != dipState){
    if(state >= 8 ){
      //vTaskSuspend(xHandleTempHumid);
      //vTaskSuspend(xHandleStepper);
      currentState = 5;
      //blockState = true;
    }else if(state == 0){
      currentState = 1;
      if(blockState == true){
      vTaskResume(xHandleTempHumid);
      vTaskResume(xHandleStepper);
      }
    }else if(state == 1){
      currentState = 2;
      vTaskResume(xHandleTempHumid);
      vTaskResume(xHandleStepper);
    }else if(state == 2 || state == 3){
      currentState = 3;
      vTaskResume(xHandleTempHumid);
      vTaskResume(xHandleStepper);
    }else if(state >3 && state <7){
      currentState = 4;
      vTaskResume(xHandleTempHumid);
      vTaskResume(xHandleStepper);
    }
    }
    vTaskDelay(500);
  }
}
/*  TempHumidController has Three possible states send Humidity , send Temp or block
void tempHumidController( void *pvParameters){
  (void) pvParameters;
  double temperature;
  double humidity;
  for(;;){
  humidity = readSensor(&temperature);
  if(currentState == 0){
    tempHumidVal = humidity;
  }else if(currentState == 1){
    tempHumidVal = temperature;
  }
  vTaskDelay(200);
  }
}*/

// Stepper has four possible states Moving to certain val, rotate clockwise, rotate counter
// clockwise or block 
void stepperController( void *pvParameters){
  (void) pvParameters;
  int currentStep = 0 ;
  int targetStep = 0, stepDiff = 0,i;
  bool clockwise;
  for ( ; ; ){
    OneStep(clockwise);
    vTaskDelay(1);
           /*// Serial.println(tempHumidVal);
    if(currentState == 0 || currentState == 1){
      targetStep = (int)  degreeToStep(tempHumidVal);
      stepDiff = targetStep - currentStep;
      if(stepDiff <0 ){
      clockwise = true;
      stepDiff = stepDiff * -1;
      //Serial.println(targetStep);
      }else{
        clockwise = false;
      }
      for(i = 0; i< targetStep ; i++){
        OneStep(clockwise);
        if(clockwise){
          currentStep = (currentStep-1)%2048;
          vTaskDelay(4);
        }else{
      currentStep = (currentStep+1)%2048;
      }
        vTaskDelay(4);
      }
    }else if(currentState == 2){
      clockwise = true;
      while(currentState == 2){
        OneStep(clockwise);
        currentStep = (currentStep-1)%2048;
      }
    }else if(currentState == 3){
      clockwise = false;
      while(currentState == 3){
        OneStep(clockwise);
        currentStep = (currentStep +1 )%2048;
      }
    }else if(currentState == 4){
      
    }*/
  }
}
void dipswitchController( void *pvParameters){
  (void) pvParameters;
  int state = 0;
  int tmp = 0,tmp2 = 0;
  int stateCurr = 0; 
  for(;;){
    // Check Dip 0 & 1 using bitwise operations if 
    tmp = PINB;
    tmp2 = (tmp &1)| ((tmp&4)>>1);
    stateCurr = stateCurr | tmp2;
    //Check Dip 2,3,4,5
    tmp = PINL;
    tmp2 = (( tmp & 1 ) <<2) | ((tmp &4) << 1) | (tmp & 16) |((tmp&64) >>1);
    stateCurr = stateCurr | tmp2;   
    // Check Dip 6,7
    tmp = PING;
    tmp2 = ((tmp & 1) << 6) | ((tmp &4) <<5);
    stateCurr = stateCurr | tmp2;

    if(stateCurr != state){
      dipState = stateCurr;
      state = stateCurr;
    }
    // Reset current state variable
    stateCurr = 0;
    vTaskDelay(dipDelay);
  }
}
void displayTask( void *pvParameters){
        struct SSDStruct * recvMsg;
        int left = -1;
        int right = -1;
        for(;;){
          if(defaultDisp.displayTime == 0 ){
          if( xQueueReceive( xQueue, &( recvMsg ), ( TickType_t ) 0 ) ){
        right = recvMsg->rightDigit;
        left = recvMsg->leftDigit;
        defaultDisp.displayTime = recvMsg->displayTime;
        }
          }
          // Remove this temp sol
          if(left ==-1 || right == -1){
            left = defaultDisp.leftDigit;
            right = defaultDisp.rightDigit;
          }
      setLEDS(left,false);
      vTaskDelay(0);
      setLEDS(right,true);
      vTaskDelay(0); 
        }
}

// Functions 

// This function sets 7segment representation of num 
void setLEDS(int num, bool right){
  PORTH &= B10000111;
  digitalWrite(SevenSegA,LOW);
  digitalWrite(SevenSegB,LOW);
  digitalWrite(SevenSegG,LOW);
if(right){
  digitalWrite(SevenSegCC1, HIGH);
  digitalWrite(SevenSegCC2, LOW);
}else{
  digitalWrite(SevenSegCC1, LOW);
  digitalWrite(SevenSegCC2, HIGH);
}
  PORTG |= ( hexSSD[num] & 1 )<< 5;
  PORTE |= (hexSSD[num] & 2) << 2;
  PORTH |= (hexSSD[num] & 60) << 1;
  PORTB |= (hexSSD[num] & 64) >> 2;
  }
  //Function converts int value into format that will be able to be shown on 7 seg display if value is 255
  // or smaller if value is larger then value Er value will be returned to be displayed

int degreeToStep(int val){
  int value;
  if(val > 0){
  value = (val * 57)/10;
  }else{
    value = 0;
  }
  Serial.println(value);

  return value;
}
int getState(int dipSwitchState){
  int currentDip = 128,i;
  for(i = 0 ; i< 8; i++){
    if((currentDip >> i & dipSwitchState ) == 0){
      break;
    }
  }
  if(8-i != 5){
    return 8-i;
  } else{
    //check second pin too
    return i;
  }
}
void intToSSD(int value ){
  SSDStruct myStruct;
  if(value > 255 ){
    myStruct.leftDigit = SSDLetterE;
    myStruct.rightDigit = SSDLetterR;
  }else{
    myStruct.leftDigit = value/16;
    myStruct.rightDigit = value % 16;
  }
}
void twoValToSSD(int leftVal, int rightVal){
  SSDStruct myStruct;
  if(leftVal > 15 || rightVal > 15 ){
    myStruct.leftDigit = SSDLetterE;
    myStruct.rightDigit = SSDLetterR;
  }else{
    myStruct.leftDigit = leftVal;
    myStruct.rightDigit = rightVal;
  }
}
// This function was written by TI 
double readSensor(double* temperature)
{
  //holds 2 bytes of data from I2C Line
  uint8_t Byte[4];
  //holds the total contents of the temp register
  uint16_t temp;
  //holds the total contents of the humidity register
  uint16_t humidity;
  //Point to device 0x40 (Address for HDC1080)
  Wire.beginTransmission(0x40);
  //Point to register 0x00 (Temperature Register)
  Wire.write(0x00);
  //Relinquish master control of I2C line
  //pointing to the temp register triggers a conversion
  Wire.endTransmission();
  //delay to allow for sufficient conversion time
  delay(20);
  //Request four bytes from registers
  Wire.requestFrom(0x40, 4);
  vTaskDelay(1);
  //If the 4 bytes were returned sucessfully
  if (4 <= Wire.available())
  {
    //take reading
    //Byte[0] holds upper byte of temp reading
    Byte[0] = Wire.read();
    //Byte[1] holds lower byte of temp reading
    Byte[1] = Wire.read();
    //Byte[3] holds upper byte of humidity reading
    Byte[3] = Wire.read();
    //Byte[4] holds lower byte of humidity reading
    Byte[4] = Wire.read();
    //Combine the two bytes to make one 16 bit int
    temp = (((unsigned int)Byte[0] <<8 | Byte[1]));
    //Temp(C) = reading/(2^16)*165(C) - 40(C)
    *temperature = (double)(temp)/(65536)*165-40;
   //Combine the two bytes to make one 16 bit int
    humidity = (((unsigned int)Byte[3] <<8 | Byte[4]));
    //Humidity(%) = reading/(2^16)*100%
    return (double)(humidity)/(65536)*100;
  }
}
// Written By Nikodem Bartnik - nikodembartnik.pl 
// I modified it to use in this project
void OneStep(bool dir){
    if(dir){
switch(step_number){
  case 0:
  digitalWrite(STEPPER_PIN_1, HIGH);
  digitalWrite(STEPPER_PIN_2, LOW);
  digitalWrite(STEPPER_PIN_3, LOW);
  digitalWrite(STEPPER_PIN_4, LOW);
  break;
  case 1:
  digitalWrite(STEPPER_PIN_1, LOW);
  digitalWrite(STEPPER_PIN_2, HIGH);
  digitalWrite(STEPPER_PIN_3, LOW);
  digitalWrite(STEPPER_PIN_4, LOW);
  break;
  case 2:
  digitalWrite(STEPPER_PIN_1, LOW);
  digitalWrite(STEPPER_PIN_2, LOW);
  digitalWrite(STEPPER_PIN_3, HIGH);
  digitalWrite(STEPPER_PIN_4, LOW);
  break;
  case 3:
  digitalWrite(STEPPER_PIN_1, LOW);
  digitalWrite(STEPPER_PIN_2, LOW);
  digitalWrite(STEPPER_PIN_3, LOW);
  digitalWrite(STEPPER_PIN_4, HIGH);
  break;
} 
  }else{
    switch(step_number){
  case 0:
  digitalWrite(STEPPER_PIN_1, LOW);
  digitalWrite(STEPPER_PIN_2, LOW);
  digitalWrite(STEPPER_PIN_3, LOW);
  digitalWrite(STEPPER_PIN_4, HIGH);
  break;
  case 1:
  digitalWrite(STEPPER_PIN_1, LOW);
  digitalWrite(STEPPER_PIN_2, LOW);
  digitalWrite(STEPPER_PIN_3, HIGH);
  digitalWrite(STEPPER_PIN_4, LOW);
  break;
  case 2:
  digitalWrite(STEPPER_PIN_1, LOW);
  digitalWrite(STEPPER_PIN_2, HIGH);
  digitalWrite(STEPPER_PIN_3, LOW);
  digitalWrite(STEPPER_PIN_4, LOW);
  break;
  case 3:
  digitalWrite(STEPPER_PIN_1, HIGH);
  digitalWrite(STEPPER_PIN_2, LOW);
  digitalWrite(STEPPER_PIN_3, LOW);
  digitalWrite(STEPPER_PIN_4, LOW);
} 
  }
step_number++;
  if(step_number > 3){
    step_number = 0;
    //steps++;
  }
}