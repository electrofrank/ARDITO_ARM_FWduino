#include <Dynamixel2Arduino.h>
#include <AccelStepper.h>
#include <Ethernet.h>
#include <PubSubClient.h>

#include <AMT222.h>
#include <SPI.h>


#define DXL_SERIAL   Serial2
#define DEBUG_SERIAL Serial

#define DEBUG 1

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

const uint8_t DXL_1 = 1; //wrist 1 joint id
const uint8_t DXL_2 = 2; //wrist 2 joint id
const uint8_t DXL_3 = 3; //wrist 2 joint id


const float dyn_max_speed = 65;
float dyn_speed_req[3]; //actual joint position, degree

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

using namespace ControlTableItem;

////////////////////// Ethernet & MQTT /////////////////////////

//ethernet card mac address
byte mac[] = {  0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02};

IPAddress server(10, 1, 0, 12);

EthernetClient ethClient;

PubSubClient client(ethClient);

///////////////// Timing Parameters ////////////////////////////////
unsigned long previousMicros = 0;
unsigned long previous_ENCODER_Micros = 0;// will store last time LED was updated
unsigned long cycle = 0;

const long STEPPER_interval = 100; //0.1 ms 10000 Hz   // interval at which to blink (milliseconds)
const long ENCODER_interval = 100000; //100 ms 10 Hz   // interval at which to blink (milliseconds)


void setup()
{

  //Initialize the UART serial connection for debugging
  if (DEBUG)
    DEBUG_SERIAL.begin(115200); //DEBUG SERIAL
  DEBUG_SERIAL.println("Ardito Arm Controller\n");

  //Set the modes for the SPI IO
  pinMode(SPI_SCLK, OUTPUT);
  pinMode(SPI_MOSI, OUTPUT);
  pinMode(SPI_MISO, INPUT);
  // SPI ENCODER CS PINS
  ENC_pin[0] = 44;
  ENC_pin[1] = 40;
  ENC_pin[2] = 36;


  SHOULDER_STEPPER.setMaxSpeed(4000);
  SHOULDER_STEPPER.setAcceleration(1000.0);

  ELBOW_STEPPER.setMaxSpeed(4000);
  ELBOW_STEPPER.setAcceleration(1000.0);

  BASE_STEPPER.setMaxSpeed(4000);
  BASE_STEPPER.setAcceleration(1000.0);

  PINZA_STEPPER.setMaxSpeed(2000);
  PINZA_STEPPER.setAcceleration(300.0);
  //Get the CS line high which is the default inactive state

  for (int i = 0; i < 3; i++) {
    pinMode(ENC_pin[i], OUTPUT);
    digitalWrite(ENC_pin[i], HIGH);
    delay(10);
  }
  //SPI.setClockDivider(SPI_CLOCK_DIV32);    // 500 kHz

  //start SPI bus

  delay(1000);

  //mqtt configuration

  client.setServer(server, 1883);
  client.setCallback(callback);

  //  //ethernet configuration
  Ethernet.init(10);  // Most Arduino shields
  Ethernet.begin(mac);

  SPI.begin();
  // start half-uplex uart
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  //Ping DYNAMIXEL
  dxl.ping(DXL_1);
  dxl.ping(DXL_2);
  dxl.ping(DXL_3);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_1);
  dxl.setOperatingMode(DXL_1, OP_VELOCITY);
  dxl.torqueOn(DXL_1);

  dxl.torqueOff(DXL_2);
  dxl.setOperatingMode(DXL_2, OP_VELOCITY);
  dxl.torqueOn(DXL_2);

  dxl.torqueOff(DXL_3);
  dxl.setOperatingMode(DXL_3, OP_VELOCITY);
  dxl.torqueOn(DXL_3);
}


void loop()
{
  if (!client.connected()) {
    reconnect();
  }

  unsigned long currentMicros = micros();

  if (currentMicros - previousMicros >= STEPPER_interval) {
    // save the last time you blinked the LED
    client.loop();

    moveARMStepper(joint_speed_req[0], joint_speed_req[1], joint_speed_req[2]);

    move_EndEffector(pinza_speed_req);

    move_WristDinamixel(dyn_speed_req[0], dyn_speed_req[1], dyn_speed_req[2]);

    previousMicros = currentMicros;
  }

}









void ethernet_init() {
  Serial.println("Initialize Ethernet with DHCP:");
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    } else if (Ethernet.linkStatus() == LinkOFF) {
      Serial.println("Ethernet cable is not connected.");
    }
    // no point in carrying on, so do nothing forevermore:
    while (true) {
      delay(1);
    }
  }
  // print your local IP address:
  Serial.print("My IP address: ");
  Serial.println(Ethernet.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {

  payload[length] = '\0';
  String s = String((char*)payload);

  float f = s.toFloat();

  //parse mqtt data into right varible
  if (strcmp(topic, "ARM/JOINT/1") == 0) {


    if (f < -0.3 || f > 0.3)
      joint_speed_req[0] = f;

    else
      joint_speed_req[0] = 0;
  }

  if (strcmp(topic, "ARM/JOINT/2") == 0) {


    if (f < -0.3 || f > 0.3)
      joint_speed_req[1] = f;

    else
      joint_speed_req[1] = 0;
  }

  if (strcmp(topic, "ARM/JOINT/3") == 0) {


    if (f < -0.2 || f > 0.2)
      joint_speed_req[2] = f;

    else
      joint_speed_req[2] = 0;
  }

  if (strcmp(topic, "ARM/EE") == 0) {
    //Serial.println("End effector req recevied");
    if (f < -0.3 || f > 0.3)
      pinza_speed_req = f;

    else
      pinza_speed_req = 0;
  }

  if (strcmp(topic, "ARM/EN") == 0) {
    if (f < -0.3 || f > 0.3)
      arm_enable = f;
    else
      arm_enable = 0;
  }

  if (strcmp(topic, "WRIST/DYN_1") == 0) {
    if (f < -0.3 || f > 0.3)
      dyn_speed_req[0] = f;
    else
      dyn_speed_req[0] = 0;
  }

  if (strcmp(topic, "WRIST/DYN_2") == 0) {
    if (f < -0.3 || f > 0.3)
      dyn_speed_req[1] = f;
    else
      dyn_speed_req[1] = 0;
  }

  if (strcmp(topic, "WRIST/DYN_3") == 0) {
    if (f < -0.3 || f > 0.3)
      dyn_speed_req[2] = f;
    else
      dyn_speed_req[2] = 0;
  }

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ARM")) {
      Serial.println("connected");

      // Once connected, publish an announcement...

      client.publish("ARM_PING", "hello world");
      // ... and resubscribe

      //subscribe to all useful topic
      client.subscribe("ARM/JOINT/1");
      client.subscribe("ARM/JOINT/2");
      client.subscribe("ARM/JOINT/3");
      client.subscribe("ARM/EE");
      client.subscribe("ARM/EN");
      client.subscribe("WRIST/DYN_1");
      client.subscribe("WRIST/DYN_2");
      client.subscribe("WRIST/DYN_3");






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

  BASE_STEPPER.setSpeed(speed_q3 * 4000); //SI APRE
  SHOULDER_STEPPER.setSpeed(-speed_q2 * 4000);
  ELBOW_STEPPER.setSpeed(speed_q1 * 4000); //SI APRE

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
  
  // Set Goal Velocity using RPM
  dxl.setGoalVelocity(DXL_1, s1*dyn_max_speed, UNIT_RPM);
  dxl.setGoalVelocity(DXL_2, s2*dyn_max_speed, UNIT_RPM);
  dxl.setGoalVelocity(DXL_2, s3*dyn_max_speed, UNIT_RPM);
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
