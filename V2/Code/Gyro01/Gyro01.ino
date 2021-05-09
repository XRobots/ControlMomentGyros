#include <Dynamixel2Arduino.h>
#include <PID_v1.h>
#include <Servo.h>

unsigned long currentMillis;
unsigned long previousMillis;

Servo myservo;  // create servo object to control a servo

double Pk1 = 1.8;  
double Ik1 = 22;
double Dk1 = 0.09;

double SetpointAccum;
double Output2;

double Setpoint1, Input1, Output1;    // PID variables
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);    // PID Setup

float pot;
float IMU;

#define DXL_SERIAL   Serial
#define DEBUG_SERIAL Serial
const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN 

const uint8_t DXL_ID = 1;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:

  PID1.SetMode(AUTOMATIC);              
  PID1.SetOutputLimits(-220000, 220000);
  PID1.SetSampleTime(10);

  pinMode(A0, INPUT);
  myservo.attach(3);  // attaches the servo on pin 3 to the servo object
  
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);
  //while(!DEBUG_SERIAL);

  Serial3.begin(115200); // read IMU data

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.torqueOn(DXL_ID);

  // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 500);
}

void loop() {

    currentMillis = millis();

    if (currentMillis - previousMillis >= 10) {  // start timed loop
          previousMillis = currentMillis;

        IMU = Serial3.parseFloat();       // read IMU    
        if (Serial3.read() == '\n') {     // end of IMU data       
        }
    
        pot = analogRead(A0);
        pot = map(pot,0,1023,800,1800);
        myservo.write(pot);               // drive VESCs

        SetpointAccum = SetpointAccum + Output1/1500;

        SetpointAccum = constrain(SetpointAccum,-0.5,0.5);
        
    
        Setpoint1 = (SetpointAccum)+0.6;    // fine tune cetre point
        Input1 = IMU;       // use IMU over serial as input
        PID1.Compute();     // compute PID output
    
    
        Output1 = constrain(Output1, -35, 35);    // make sure we don't turn gyros further than 25'
        Output2 = Output1+180;                    // use the centre part of the Dynamixel rotation

        //Serial.println(SetpointAccum);                    // debug if required    
    
        dxl.setGoalPosition(DXL_ID, Output2, UNIT_DEGREE);    // drive Dynamixel to move gyros

    }   // end of 10ms loop

  

}
