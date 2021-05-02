#include <Dynamixel2Arduino.h>
#include <PID_v1.h>

double Pk1 = 0.3;  
double Ik1 = 6.2;
double Dk1 = 0.03;

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

    IMU = Serial3.parseFloat();

    if (Serial3.read() == '\n') {
      
    }

    //pot = analogRead(A0);

    //pot = (pot - 512) / 100;

    //Serial.println(pot);

    IMU = IMU -0.9;


    

    Setpoint1 = -0.25;    // fine tune cetre point
    Input1 = IMU;
    PID1.Compute();


    Output1 = constrain(Output1, -25, 25);
    Output1 = Output1+180;

      



    dxl.setGoalPosition(DXL_ID, Output1, UNIT_DEGREE);

    delay(10);


  

}
