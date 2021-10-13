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

uint8_t DXL[3];

const float dyn_max_speed = 20.0;

float dyn_speed_req[3]; //actual joint position, degree

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

DYNAMIXEL::InfoFromPing_t ping_info[32];

using namespace ControlTableItem;

////////////////////// Ethernet & MQTT /////////////////////////

//ethernet card mac address
byte mac[] = {  0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02};

IPAddress server(10, 0, 0, 110);

EthernetClient ethClient;

PubSubClient client(ethClient);

///////////////// Timing Parameters ////////////////////////////////
unsigned long previousMicros = 0;
unsigned long previousARMMicros = 0;// will store last time LED was updated
unsigned long cycle = 0;

const long STEPPER_interval = 50000; //1 ms 1000 Hz   // interval at which to blink (milliseconds)
const long ENCODER_interval = 500; //1000 us 1ms 1 KHz   // interval at which to blink (milliseconds)


void setup()
{

  //Initialize the UART serial connection for debugging
  if (DEBUG) {
    DEBUG_SERIAL.begin(115200); //DEBUG SERIAL
    DEBUG_SERIAL.println("Ardito Arm Controller\n");
  }

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

  dxl.torqueOff(DXL[0]);
  dxl.setOperatingMode(DXL[0], OP_VELOCITY);
  dxl.torqueOn(DXL[0]);

  dxl.torqueOff(DXL[1]);
  dxl.setOperatingMode(DXL[1], OP_VELOCITY);
  dxl.torqueOn(DXL[1]);

  
  
    }




void loop()
{
  if (!client.connected()) {
    reconnect();
  }
  
  
 client.loop();

  //moveARM must be called as fast as possible

  unsigned long currentMicros = micros();

  if (currentMicros - previousARMMicros >= ENCODER_interval) {
    

    moveARMStepper(joint_speed_req[0], joint_speed_req[1], joint_speed_req[2]);

    previousARMMicros = currentMicros;

  }

  if (currentMicros - previousMicros >= STEPPER_interval) {
    // save the last time you blinked the LED
    move_WristDinamixel(dyn_speed_req[0], dyn_speed_req[1], dyn_speed_req[2]);
    

    previousMicros = currentMicros;
  }


  //move_EndEffector(pinza_speed_req);


}









void ethernet_init() {
  DEBUG_SERIAL.println("Initialize Ethernet with DHCP:");
  if (Ethernet.begin(mac) == 0) {
    DEBUG_SERIAL.println("Failed to configure Ethernet using DHCP");
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      DEBUG_SERIAL.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    } else if (Ethernet.linkStatus() == LinkOFF) {
      DEBUG_SERIAL.println("Ethernet cable is not connected.");
    }
    // no point in carrying on, so do nothing forevermore:
    while (true) {
      delay(1);
    }
  }
  // print your local IP address:
  DEBUG_SERIAL.print("My IP address: ");
  DEBUG_SERIAL.println(Ethernet.localIP());
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
    dyn_speed_req[0] = f;
  }


  if (strcmp(topic, "WRIST/DYN_2") == 0) {
    dyn_speed_req[1] = f;

  }

  if (strcmp(topic, "WRIST/DYN_3") == 0) {
    dyn_speed_req[2] = f;

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
//            client.subscribe("ARM/EE");
//            client.subscribe("ARM/EN");
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

//  Serial.print(speed_q1);
//    Serial.print("  ");
//  Serial.println(speed_q2);

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
//  Serial.print(s1);
//    Serial.print("  ");
//  Serial.println(s2);

  // Set Goal Velocity using RPM
  dxl.setGoalVelocity(DXL[0], (float)(s1 * dyn_max_speed), UNIT_RPM);
  dxl.setGoalVelocity(DXL[1], (float)(s2 * dyn_max_speed), UNIT_RPM);
  //dxl.setGoalVelocity(DXL[2], s3 * dyn_max_speed, UNIT_RPM);
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
  
  Serial.println("  Try Protocol 2 - broadcast ping: ");
  Serial.flush(); // flush it as ping may take awhile... 
      
  if (uint8_t count_pinged = dxl.ping(DXL_BROADCAST_ID, ping_info, 
    sizeof(ping_info)/sizeof(ping_info[0]))) {
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
  }else{
    Serial.print("Broadcast returned no items : ");
    Serial.println(dxl.getLastLibErrCode());
  }
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
