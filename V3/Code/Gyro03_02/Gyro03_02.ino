#include <Dynamixel2Arduino.h>
#include <PID_v1.h>
#include <Servo.h>

//ODrive
#include <ODriveArduino.h>

//ODrive Objects
ODriveArduino odrive1(Serial2);

unsigned long currentMillis;
unsigned long previousMillis;

int requested_state;

Servo myservo;  // create servo object to control a servo

// Gyro PID

double Pk1 = 3.5;  
double Ik1 = 40;
double Dk1 = 0.10;

double SetpointAccum;
double Output1a;

double Setpoint1, Input1, Output1;    // PID variables
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);    // PID Setup

// wheel PID

double Pk2 = 8500;  
double Ik2 = 90000;
double Dk2 = 200;

double Setpoint2, Input2, Output2;    // PID variables
PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2 , Dk2, DIRECT);    // PID Setup

float pot;
float IMUroll;
float IMUpitch;


int sw1;    // Odrive init
float pot2;   // setpoint trim for singel balancing wheel

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

  pinMode(50, INPUT_PULLUP);    // ODrive init

  PID1.SetMode(AUTOMATIC);              
  PID1.SetOutputLimits(-220000, 220000);
  PID1.SetSampleTime(10);

  PID2.SetMode(AUTOMATIC);              
  PID2.SetOutputLimits(-60000, 60000);
  PID2.SetSampleTime(10);

  pinMode(A0, INPUT);
  myservo.attach(3);  // attaches the servo on pin 3 to the servo object
  
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);
  //while(!DEBUG_SERIAL);

  Serial2.begin(115200); // read IMU data
  Serial3.begin(115200); // read IMU data
  Serial1.begin(115200);    // ODrive

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

          pot2 = analogRead(A1);
          pot2 = (pot2 - 512)/100; 

          sw1 = digitalRead(50);                 // init ODrive
            if (sw1 == 0) {
              OdriveInit1();
            }

          IMUroll = Serial3.parseFloat();       // read IMU roll
          IMUpitch = Serial3.parseFloat();       // read IMU pitch
          if (Serial3.read() == '\n') {     // end of IMU data       
          }
      
          pot = analogRead(A0);
          pot = map(pot,0,1023,800,1800);
          myservo.write(pot);               // drive VESCs
  
          SetpointAccum = SetpointAccum + Output1/1500;  
          SetpointAccum = constrain(SetpointAccum,-0.5,0.5);

          // gyro PID
      
          Setpoint1 = (SetpointAccum)+0.6;    // fine tune cetre point
          Input1 = IMUroll;       // use IMU over serial as input
          PID1.Compute();     // compute PID output    
      
          Output1 = constrain(Output1, -35, 35);    // make sure we don't turn gyros further than 25'
          Output1a = Output1+180;                    // use the centre part of the Dynamixel rotation 
      
          dxl.setGoalPosition(DXL_ID, Output1a, UNIT_DEGREE);    // drive Dynamixel to move gyros

          // wheel PID

          Setpoint2 = pot2;
          Input2 = IMUpitch;
          PID2.Compute();



          odrive1.SetVelocity(0, Output2*-1); 

    }   // end of 10ms loop

  

}
