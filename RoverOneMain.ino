//------------------------------------------------------------------------------
//                                Libraries
//------------------------------------------------------------------------------
// WIFI
#include "ESP32_WiFi.h"
// nRF24L01 Radio
#include "RF24.h"
#include <nRF24L01.h>
#include <SPI.h>
// Time of Flight Sensor 
#include "Adafruit_VL53L0X.h"
// IMU MPU6050
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
// MQTT
#include "Esp32MQTTClient.h"
// Include Wifi access codes
#include "wifiAccessCodes.h"

#include "roverData.h"
#include "roverTelemetry.h"
#include "roverMotorCntrl.h"
#include "roverJoystickCntrl.h"
//------------------------------------------------------------------------------
/*
 *      RoverOne Debug & Drive   -- mk 1.1
 *      
 *      Hardware: 
 *      OBC:              ESP32 Dev Kit
 *      IMU:              MPU6050
 *      Front Distance:   VL53L0X
 *      Additional Radio: NRF24L01
 *      Motor Control:    2 x L297
 *      Power:            3 x 18650 cells 
 * 
 *         ----------- 
 *         |         |
 * Wheel A |A       C| Wheel C
 *         |         |
 * Wheel D |D       B| Wheel B
 *         |    v    |
 *         -----------
 *        
 *        Radio Receiver Setup:
 *        
 *        Module: NRF24L01 Radio Transmitter/Receiver
 *        Pins:
 *                SCK  -> 18
 *                MISO -> 19
 *                MOSI -> 23
 *                CSN  ->  5
 *                CE   ->  4
 *        
*/
//------------------------------------------------------------------------------
// Define Drive Modes:
//------------------------------------------------------------------------------ 
 const int DEMO_SINGLE_WHEEL = 1;
 const int DEMO_ALL_WHEEL    = 2;
 const int DEMO_REMOTE_CNTRL = 3;
 const int DEMO_AUTO_DRIVE_1 = 4;
//------------------------------------------------------------------------------
// !!! Set Drive Mode:
//------------------------------------------------------------------------------
 int  ActiveDriveMode = DEMO_AUTO_DRIVE_1;
 //------------------------------------------------------------------------------
 // Set telemetry announcements
 boolean tm_announce_drive_mode = true;

 boolean isSkipSensorInit = false;
 // Equipment
 boolean enableIMU        = true;
 boolean enableDistFront  = true;
 boolean enableRadio      = true;
 boolean enableWifi       = true;
 boolean enableMqtt       = false;
//------------------------------------------------------------------------------
// TM baud rate
const long tm_baud = 115200;
//------------------------------------------------------------------------------
// Time of flight sensor (VL53L0X)
Adafruit_VL53L0X vl53lox = Adafruit_VL53L0X();
int SHUT_VL = 12;
#define LOX1_ADDRESS 0x30
//------------------------------------------------------------------------------
// MPU6050
Adafruit_MPU6050 mpu;
//------------------------------------------------------------------------------
// Radio Communication
int CE  = 4;
int CSN = 5;
RF24 radio(CE,CSN);
const uint64_t NRF24_PIPE_ADR = 0xE8E8F0F0E1LL;
//------------------------------------------------------------------------------
// Wifi
// Initialise the WiFi and MQTT Client objects
WiFiClient wifiClient;
//------------------------------------------------------------------------------
// Driving speed in auto mode [0 255]
int autoDrivingPace = 80;

// Time since radio is without contact
long timeOffRadio = 0 ; 

// Mqtt status 
static bool hasIoTHub     = false;
// Enable MQTT debug prints
static bool isMqttDebug   = true;
//------------------------------------------------------------------------------
void setup() {
  // Setup TM serial
  Serial.begin(tm_baud);
  delay(10);
  
  Serial.println();
  Serial.println("RoverOne mk1 init ... ");
    
  // Set all the motor control pins to outputs
  pinMode(inA1, OUTPUT);
  pinMode(inA2, OUTPUT);
  pinMode(inB1, OUTPUT);
  pinMode(inB2, OUTPUT);
  pinMode(inC1, OUTPUT);
  pinMode(inC2, OUTPUT);
  pinMode(inD1, OUTPUT);
  pinMode(inD2, OUTPUT);
  
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(enC, OUTPUT);
  pinMode(enD, OUTPUT);

  ledcSetup(PWMA_Ch, PWM_Freq, PWM_Res);
  ledcAttachPin(enA, PWMA_Ch);

  ledcSetup(PWMB_Ch, PWM_Freq, PWM_Res);
  ledcAttachPin(enB, PWMB_Ch);

  ledcSetup(PWMC_Ch, PWM_Freq, PWM_Res);
  ledcAttachPin(enC, PWMC_Ch);

  ledcSetup(PWMD_Ch, PWM_Freq, PWM_Res);
  ledcAttachPin(enD, PWMD_Ch);

  // INIT WIFI
  if ( enableWifi ) {
    connectRoverWifi();
  }

  // INIT MPU6050
  if ( enableIMU && !isSkipSensorInit ){
    Serial.println("INIT: IMU Sensor MPU6050 ...");
    if ( !mpu.begin() ) {
      Serial.println("Failed to find MPU6050");
    }
  Serial.println(">> MPU6050 Sensor Online.");
  }
  
  // INIT VL53L0X 
  if (enableDistFront && !isSkipSensorInit){
      // all reset
    digitalWrite(SHUT_VL, LOW);    
    delay(10);
    // all unreset
    digitalWrite(SHUT_VL, HIGH);
    delay(10);
    Serial.println("INIT: Front Distance Sensor VL53L0X ...");
    if (!vl53lox.begin(LOX1_ADDRESS)) {
      Serial.println(F("Failed to boot VL53L0X"));
    }
    Serial.println(">> Front Distance Sensor Online."); 
  } else if (enableDistFront) {
    vl53lox.begin(LOX1_ADDRESS);
  }
 
  // INIT NRF24
  if (enableRadio) {
    Serial.println("INIT: NRF24");
    radio.begin();
    
    //set the address
    radio.openReadingPipe(0, NRF24_PIPE_ADR);
    
    //Set module as receiver
    radio.startListening();
  }

  // Init MQTT
  if ( enableMqtt) 
  {
      Serial.println("INIT: MQTT");
      if (!Esp32MQTTClient_Init((const uint8_t*)connectionString))
      {
        hasIoTHub = false;
        Serial.println("Initializing IoT hub failed.");
      }
  }
  
  // Come to life routine
  Serial.println(" >> Execute come to life routine.");
  comeToLifeRoutine();

  
  // Set all wheels >> STOP
  Serial.println(" >> Set all motor control to STOP");
  digitalWrite(inA1, HIGH);
  digitalWrite(inA2, HIGH);
  digitalWrite(inB1, HIGH);
  digitalWrite(inB2, HIGH);
  digitalWrite(inC1, HIGH);
  digitalWrite(inC2, HIGH);
  digitalWrite(inD1, HIGH);
  digitalWrite(inD2, HIGH);

  long timeOffRadio = 0 ; 

  Serial.println("... Initialization complete!");
  Serial.println();
}

void loop() {
  const int demoWheelSpeedSet = 90;
  
  if (tm_announce_drive_mode) {
    Serial.println("Enter Drive Mode");
    Serial.print("Drive mode: ");
    Serial.print(ActiveDriveMode);
    Serial.println();
    tm_announce_drive_mode = false;
  }
  
  switch (ActiveDriveMode) {
    case DEMO_SINGLE_WHEEL:
        // <Alternating single wheel (forward/backward) sequence to check wiring>
        // Wheel (A) Demo 
        Serial.println("Wheel A");
        delay(800);
        ctrlWheel_A(CMD_FORWARD , demoWheelSpeedSet);
        delay(1000);
        ctrlWheel_A(CMD_STOP    ,   0);
        delay(200);
        ctrlWheel_A(CMD_REVERSE , demoWheelSpeedSet);
        delay(1000);
        ctrlWheel_A(CMD_STOP    ,   0);
      
        // Wheel (B) Demo 
        Serial.println("Wheel B");
        delay(800);
        ctrlWheel_B(CMD_FORWARD , demoWheelSpeedSet);
        delay(1000);
        ctrlWheel_B(CMD_STOP    ,   0);
        delay(200);
        ctrlWheel_B(CMD_REVERSE , demoWheelSpeedSet);
        delay(1000);
        ctrlWheel_B(CMD_STOP    ,   0);
      
        // Wheel (C) Demo 
        Serial.println("Wheel C");
        delay(800);
        ctrlWheel_C(CMD_FORWARD , demoWheelSpeedSet);
        delay(1000);
        ctrlWheel_C(CMD_STOP    ,   0);
        delay(200);
        ctrlWheel_C(CMD_REVERSE , demoWheelSpeedSet);
        delay(1000);
        ctrlWheel_C(CMD_STOP    ,   0);
        
        // Wheel (D) Demo 
        Serial.println("Wheel D");
        delay(800);
        ctrlWheel_D(CMD_FORWARD , demoWheelSpeedSet);
        delay(1000);
        ctrlWheel_D(CMD_STOP    ,   0);
        delay(200);
        ctrlWheel_D(CMD_REVERSE , demoWheelSpeedSet);
        delay(1000);
        ctrlWheel_D(CMD_STOP    ,   0);
        break;
    case DEMO_ALL_WHEEL:
        // Wheel (A) Demo 
        ctrlWheel_A(CMD_FORWARD , 255);
        ctrlWheel_B(CMD_FORWARD , 255);
        ctrlWheel_C(CMD_FORWARD , 255);
        ctrlWheel_D(CMD_FORWARD , 255);
        delay(1000);
        ctrlWheel_A(CMD_STOP , 255);
        ctrlWheel_B(CMD_STOP , 255);
        ctrlWheel_C(CMD_STOP , 255);
        ctrlWheel_D(CMD_STOP , 255);
        delay(1000);
        ctrlWheel_A(CMD_REVERSE , 255);
        ctrlWheel_B(CMD_REVERSE , 255);
        ctrlWheel_C(CMD_REVERSE , 255);
        ctrlWheel_D(CMD_REVERSE , 255);
        delay(1000);
        ctrlWheel_A(CMD_STOP , 255);
        ctrlWheel_B(CMD_STOP , 255);
        ctrlWheel_C(CMD_STOP , 255);
        ctrlWheel_D(CMD_STOP , 255);
         delay(1000);
         break;
    case DEMO_REMOTE_CNTRL:
      // Remote Joystick Control only:
      ctrlAllWheel_Stop();
      if( radio.available() )  {
        timeOffRadio = millis() ; 
        enableJoystickCtrl(radio);
      } else {
        int switchThrTime = 2500;
        if ( (millis() - timeOffRadio) > switchThrTime ){
          setDrivingMode( DEMO_AUTO_DRIVE_1 );
          break;
        } 
      }
      if(enableDistFront){
        Serial.print("Distance Front: ");
        Serial.println(measFrontDistance());
      }
      break;
    case DEMO_AUTO_DRIVE_1:
        // Automatic drive demo with object detection 
        // Object detection with time of flight sensor @ front/center       
        if( radio.available() )  {
            setDrivingMode( DEMO_REMOTE_CNTRL );
            break;
        } else  {
            int turnDist = 200;
            volatile int frontDistance = measFrontDistance();
            
            if(enableDistFront){
              Serial.print("Distance Front: ");
              Serial.println(frontDistance);
            }
            
            if ( frontDistance < turnDist && frontDistance != -1) {
              // Perform turn
              ctrlAllWheel_Stop();
              ctrlAllWheel_Reverse(autoDrivingPace);
              delay(650);
              ctrlAllWheel_RotateRightward(autoDrivingPace);
              delay(random(200,550));
              ctrlAllWheel_Stop();
              delay(20); 
            } else {
              ctrlAllWheel_Forward(autoDrivingPace);
            }
        }
        delay(10);
        break;
    default:
        ctrlAllWheel_Stop();
        //delay(1000);
        break;
  }

}

void comeToLifeRoutine(){
  // Shake 30 Hz for 2 seconds
  shake(40,1.5);
}

void shake(float frequency, float t){
  // period in s
  float period = 1/frequency;
  
  // Number of cycles
  int cycles = (int) (t/period);
  
    for (int ii = 1; ii < cycles ; ii++){   
  
    ctrlAllWheel_Forward(150);
    delay(period/3*1000);
    ctrlAllWheel_Stop();
    delay(period/3*1000);
    ctrlAllWheel_Reverse(150);;
    delay(period/3*1000);
    
  }

}

void setDrivingMode(int newMode){
  ActiveDriveMode       = newMode;
  telemetry.drivingMode = newMode;
}

int measFrontDistance(){
  int distance = -1;
  if(enableDistFront)
  {
      VL53L0X_RangingMeasurementData_t measure;
      vl53lox.rangingTest(&measure, false);   
       
      if (measure.RangeStatus != 4) 
      {  
        distance = measure.RangeMilliMeter;
      } 
  }
  telemetry.frontDistance  = distance;
  return distance;
}

void connectRoverWifi(){
  Serial.println("INIT: WIFI");
  WiFi.begin(ssid, wifi_password); 
}



void publishTelemetry(){
  telemetry.clockTime = millis();
  
  if (hasIoTHub)
  {
    char buff[128];

    // replace the following line with your data sent to Azure IoTHub
    snprintf(buff, 128, "{\"topic\":\"iot\"}");
    
    if (Esp32MQTTClient_SendEvent(buff))
    {
      if (isMqttDebug){
        Serial.println("Sending data succeed");
      }
    }
    else if (isMqttDebug)
    {
      Serial.println("Failure...");
    }
  }
}
