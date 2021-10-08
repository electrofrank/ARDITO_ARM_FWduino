/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include <Dynamixel2Arduino.h>









  #define DXL_SERIAL   Serial2
  #define DEBUG_SERIAL Serial
  
  const uint8_t DXL_DIR_PIN = 26; // DYNAMIXEL Shield DIR PIN

 

const uint8_t DXL_1 = 1;
const uint8_t DXL_2 = 2;


const float DXL_PROTOCOL_VERSION = 2.0;

const float max_speed = 65;
float req_speed = 0;
int flag = 1;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
  
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);
  
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_1);
  dxl.ping(DXL_2);
  
  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_1);
  dxl.setOperatingMode(DXL_1, OP_VELOCITY);
  dxl.torqueOn(DXL_1);

    dxl.torqueOff(DXL_2);
  dxl.setOperatingMode(DXL_2, OP_VELOCITY);
  dxl.torqueOn(DXL_2);
}

void loop() {


  // Set Goal Velocity using RPM
  dxl.setGoalVelocity(DXL_1, req_speed, UNIT_RPM);
  dxl.setGoalVelocity(DXL_2, req_speed, UNIT_RPM);

  DEBUG_SERIAL.print(dxl.getPresentVelocity(DXL_1, UNIT_RPM));
    DEBUG_SERIAL.print(",");

  DEBUG_SERIAL.println(dxl.getPresentVelocity(DXL_2, UNIT_RPM));
 
req_speed = req_speed + 1*flag;

if(req_speed >= max_speed) flag = -1;

if(req_speed <= -max_speed) flag = 1;

    delay(10);
}
