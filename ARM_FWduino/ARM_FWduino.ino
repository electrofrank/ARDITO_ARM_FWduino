#include <Dynamixel2Arduino.h>
#include <AccelStepper.h>
#include <Ethernet.h>
#include <PubSubClient.h>

#include <AMT222.h>
#include <SPI.h>


// uncomment for use with stm32h7
//HardwareSerial Serial2(PD10, PA9);

#define DXL_SERIAL   Serial2
#define DEBUG_SERIAL Serial
#define DEBUG 1
#define MICROSTEP 1
#define MULTIPLIER 90*MICROSTEP

//////////// Encoders /////////////////////////
int ENC_pin[3];

float joint_pos_act[3]; //actual joint position, degree

//////////// Steppers ////////////////////////
//base stepper
#define BASE_EN    27
#define BASE_DIR   29
#define BASE_STEP  31
//shoulder stepper
#define SHOULDER_EN  35
#define SHOULDER_DIR  37
#define SHOULDER_STEP  39
//elbow stepper
#define ELBOW_EN  43
#define ELBOW_DIR  45
#define ELBOW_STEP  47
//pinza stepper
#define PINZA_EN 4
#define PINZA_DIR 2
#define PINZA_STEP 3

AccelStepper BASE_STEPPER(1, BASE_STEP, BASE_DIR);
AccelStepper SHOULDER_STEPPER(1, SHOULDER_STEP, SHOULDER_DIR);
AccelStepper ELBOW_STEPPER(1, ELBOW_STEP, ELBOW_DIR);
AccelStepper PINZA_STEPPER(1, PINZA_STEP, PINZA_DIR);

float joint_speed_req[3]; //actual joint position, degree
float pinza_speed_req = 0;
float arm_enable = 0;

////////////////////// Dynamixel ///////////////////////////////
const uint8_t DXL_DIR_PIN = 26; // Tri-State Buffer DIR PIN
const float DXL_PROTOCOL_VERSION = 2.0;

uint8_t DXL[3];

const float dyn_max_speed = 5.0;

float dyn_speed_req[3]; //actual joint position, degree

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

DYNAMIXEL::InfoFromPing_t ping_info[32];

using namespace ControlTableItem;

////////////////////// Ethernet & MQTT /////////////////////////

//ethernet card mac address
byte mac[] = {  0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02};

IPAddress server(10, 1, 0, 101);

EthernetClient ethClient;

PubSubClient client(ethClient);

///////////////// Timing Parameters ////////////////////////////////
unsigned long previousMicros = 0;
unsigned long previousARMMicros = 0;// will store last time LED was updated
unsigned long cycle = 0;

const long WRIST_interval = 50000; //50 ms 20 Hz   // interval at which to blink (milliseconds)
const long ARM_interval = 200; //1000 us 1ms 1 KHz   // interval at which to blink (milliseconds)


void setup()
{

  //Initialize the UART serial connection for debugging

  if (DEBUG) {
//    Serial.setRx(PD9); // using pin name PY_n
//    Serial.setTx(PD8); // using pin number PYn
    DEBUG_SERIAL.begin(115200); //DEBUG SERIAL
    DEBUG_SERIAL.println("Ardito Arm Controller\n");
  }

  // SPI ENCODER CS PINS
  ENC_pin[0] = 44;
  ENC_pin[1] = 40;
  ENC_pin[2] = 36;


  SHOULDER_STEPPER.setMaxSpeed(2000);
  SHOULDER_STEPPER.setAcceleration(500.0);

  ELBOW_STEPPER.setMaxSpeed(2000);
  ELBOW_STEPPER.setAcceleration(500.0);

  BASE_STEPPER.setMaxSpeed(2000);
  BASE_STEPPER.setAcceleration(500.0);

  PINZA_STEPPER.setMaxSpeed(2000);
  PINZA_STEPPER.setAcceleration(300.0);
  //mqtt configuration

  client.setServer(server, 1883);
  client.setCallback(callback);


  //  //ethernet configuration
  Ethernet.init(10);  // Most Arduino shields
  ethernet_init();

  SPI.begin();

  ///////// DYNAMIXEL INITIALIZATION //////////////
  // ID DEFINITON
  DXL[0] = 1;
  DXL[1] = 2;
  DXL[2] = 3;

  dyn_speed_req[0] = 0;
  dyn_speed_req[1] = 0;
  dyn_speed_req[2] = 0;

  // start half-uplex uart
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // INITIALIZATION LOOP
  //Ping DYNAMIXEL
FindServos();
//
//  dxl.torqueOff(DXL[0]);
//  dxl.setOperatingMode(DXL[0], OP_VELOCITY);
//  dxl.torqueOn(DXL[0]);
//
//  dxl.torqueOff(DXL[1]);
//  dxl.setOperatingMode(DXL[1], OP_VELOCITY);
//  dxl.torqueOn(DXL[1]);
//
//  dxl.torqueOff(DXL[2]);
//  dxl.setOperatingMode(DXL[2], OP_VELOCITY);
//  dxl.torqueOn(DXL[2]);

}




void loop()
{
  //Serial.println("loop");
  
  if (!client.connected()) {
    reconnect();
  }

  client.loop();

  //moveARM must be called as fast as possible

  unsigned long currentMicros = micros();

  if (currentMicros - previousARMMicros >= ARM_interval) {

    previousARMMicros = currentMicros;

    moveARMStepper(joint_speed_req[0], joint_speed_req[1], joint_speed_req[2]);
  }


  if (currentMicros - previousMicros >= WRIST_interval) {
    // save the last time you blinked the LED
    move_WristDinamixel(dyn_speed_req[0], dyn_speed_req[1], dyn_speed_req[2]);


    previousMicros = currentMicros;
  }

}









void ethernet_init() {

IPAddress ip(10,1,0,80);
  
  DEBUG_SERIAL.println("Initialize Ethernet...");
  Ethernet.begin(mac,ip);
  
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      DEBUG_SERIAL.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
        while (true) {
      delay(1);
        }
   } 

    // no point in carrying on, so do nothing forevermore:
  
  // print your local IP address:
  DEBUG_SERIAL.print("Assigned IP address: ");
  DEBUG_SERIAL.println(Ethernet.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {

  payload[length] = '\0';
  String s = String((char*)payload);
  float f = s.toFloat();

  //parse mqtt data into right varible
  if (strcmp(topic, "ARM/JOINTS") == 0) {
    parseString(s);
    //Serial.println(s);
  }

  if (strcmp(topic, "WRIST/JOINTS") == 0) {
    parseStringWRIST(s);
  }

//  if (strcmp(topic, "ARM/BASE_ANGLE") == 0) {
//    //esegue una rotazione di n gradi il giunto di base
//    Serial.println("New base joint position received");
//    //PositionControl(f);
//  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ARM")) {
      Serial.println("connected");

      // Once connected, publish an announcement...

      client.publish("ARM_PING", "hello Ardito");
      client.publish("ARM/BASE_ANGLE", "0");

      // ... and resubscribe

      //subscribe to all useful topic
      client.subscribe("ARM/JOINTS");
      client.subscribe("WRIST/JOINTS");
      client.subscribe("ARM/BASE_ANGLE");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void moveARMStepper(float speed_q1, float speed_q2, float speed_q3) {

  BASE_STEPPER.setSpeed(speed_q3 * 1000); //SI APRE
  SHOULDER_STEPPER.setSpeed(-speed_q2 * 1000);
  ELBOW_STEPPER.setSpeed(speed_q1 * 1000); //SI APRE

  BASE_STEPPER.runSpeed(); //SI APRE
  SHOULDER_STEPPER.runSpeed();
  ELBOW_STEPPER.runSpeed(); //SI APRE

}

void move_EndEffector(float speed_ee) {
  PINZA_STEPPER.setSpeed(speed_ee * 500); //SI APRE
  PINZA_STEPPER.runSpeed(); //SI APRE
}

void move_WristDinamixel(float s1, float s2, float s3) {
  //map input (-1,1) to max velocity
//  Serial.print(s1);
//  Serial.print("  ");
//  Serial.println(s2);

  // Set Goal Velocity using RPM
  dxl.setGoalVelocity(DXL[0], (float)(s1 * dyn_max_speed), UNIT_RPM);
  dxl.setGoalVelocity(DXL[1], (float)(s2 * dyn_max_speed), UNIT_RPM);
  dxl.setGoalVelocity(DXL[2], (float)(s3 * dyn_max_speed), UNIT_RPM);
}

void getARM_joint_pos() {
  uint16_t encoderPosition;
  //let's also create a variable where we can count how many times we've tried to obtain the position in case there are errors
  int i, j;

  for (i = 0; i < 3; i++) {
    digitalWrite(ENC_pin[i], LOW);

    joint_pos_act[i] = (360 / 4095.0) * (getPositionSPI(ENC_pin[i], RES12));
    delay(1);
    digitalWrite(ENC_pin[i], HIGH);

    Serial.print("Encoder ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(joint_pos_act[i]);
    Serial.print(" - ");
  }

  Serial.println("\n");
}

void FindServos(void) {

  //Serial.println("  Try Protocol 2 - broadcast ping: ");
  Serial.flush(); // flush it as ping may take awhile...

  if (uint8_t count_pinged = dxl.ping(DXL_BROADCAST_ID, ping_info,
                                      sizeof(ping_info) / sizeof(ping_info[0]))) {
    Serial.print("Detected Dynamixel : \n");
    for (int i = 0; i < count_pinged; i++)
    {
      Serial.print("    ");
      Serial.print(ping_info[i].id, DEC);
      Serial.print(", Model:");
      Serial.print(ping_info[i].model_number);
      Serial.print(", Ver:");
      Serial.println(ping_info[i].firmware_version, DEC);
      //g_servo_protocol[i] = 2;
    }
  } else {
    Serial.print("Broadcast returned no items : ");
    Serial.println(dxl.getLastLibErrCode());
  }
}



void parseString(String s) {

  //find commas
  int slength = s.length();
  int firstIndex = s.indexOf(',');
  int secondIndex = s.indexOf(',', firstIndex + 1);

  float temp1 = (s.substring(0, firstIndex)).toFloat();
  float temp2 = (s.substring(firstIndex + 1 , secondIndex)).toFloat();
  float temp3 = (s.substring(secondIndex + 1 , slength)).toFloat();

  if (temp1 < -0.3 || temp1 > 0.3)
    joint_speed_req[0] = temp1;
  else
    joint_speed_req[0] = 0;

  if (temp2 < -0.3 || temp2 > 0.3)
    joint_speed_req[1] = temp2;
  else
    joint_speed_req[1] = 0;

  if (temp3 < -0.3 || temp3 > 0.3)
    joint_speed_req[2] = temp3;
  else
    joint_speed_req[2] = 0;

}

void parseStringWRIST(String s) {

  //find commas
  int slength = s.length();
  int firstIndex = s.indexOf(',');
  int secondIndex = s.indexOf(',', firstIndex + 1);

  float temp1 = (s.substring(0, firstIndex)).toFloat();
  float temp2 = (s.substring(firstIndex + 1 , secondIndex)).toFloat();
  float temp3 = (s.substring(secondIndex + 1 , slength)).toFloat();

  if (temp1 < -0.3 || temp1 > 0.3)
    dyn_speed_req[0] = temp1;
  else
    dyn_speed_req[0] = 0;

  if (temp2 < -0.3 || temp2 > 0.3)
    dyn_speed_req[1] = temp2;
  else
    dyn_speed_req[1] = 0;

  if (temp3 < -0.3 || temp3 > 0.3)
    dyn_speed_req[2] = temp3;
  else
    dyn_speed_req[2] = 0;

}


//questa funzione riceve un angolo tramite mqtt e pilota gli stepper in OpenLoop
void PositionControl(int angle) {
  int steps = angle*MULTIPLIER;
  BASE_STEPPER.setAcceleration(200);
  BASE_STEPPER.setSpeed(100);
BASE_STEPPER.runToNewPosition(steps);
}











//void disable_stepper {
//      digitalWrite(BASE_EN, HIGH);
//      digitalWrite(SHOULDER_EN, HIGH);
//      digitalWrite(ELBOW_EN, HIGH);
//      digitalWrite(PINZA_EN, HIGH);
//}


/*
   if(arm_enable == 1.00) { //enable stepper driver
  digitalWrite(BASE_EN, HIGH);
  digitalWrite(SHOULDER_EN, HIGH);
  digitalWrite(ELBOW_EN, HIGH);
  digitalWrite(PINZA_EN, HIGH);
  }

  if(arm_enable == 0.00) { //disable stepper driver
  digitalWrite(BASE_EN,LOW);
  digitalWrite(SHOULDER_EN,LOW);
  digitalWrite(ELBOW_EN,LOW);
  digitalWrite(PINZA_EN,LOW);
  }
*/
