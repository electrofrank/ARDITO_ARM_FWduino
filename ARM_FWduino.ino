
#include <AccelStepper.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>

#include "AMT222.h"


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
////////////////////// Ethernet & MQTT /////////////////////////

//ethernet card mac address
byte mac[] = {  0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02};

IPAddress server(10,1,0,12);

EthernetClient ethClient;

PubSubClient client(ethClient);

///////////////// Timing Parameters ////////////////////////////////
unsigned long previousMicros = 0;       // will store last time LED was updated

const long interval = 10; //1 ms 100 Hz   // interval at which to blink (milliseconds)


void setup()
{

  //Initialize the UART serial connection for debugging
  Serial.begin(115200); //DEBUG SERIAL

  Serial.println("Ardito Arm Controller\n");  
  
  Serial1.begin(115200); //WRIST BOARD SERIAL

  
  //Set the modes for the SPI IO
  pinMode(SPI_SCLK, OUTPUT);
  pinMode(SPI_MOSI, OUTPUT);
  pinMode(SPI_MISO, INPUT);
  // SPI ENCODER CS PINS
  ENC_pin[0] = 44;
  ENC_pin[1] = 40;
  ENC_pin[2] = 36;
  
  for (int i = 0; i < 3; i++) {
    pinMode(ENC_pin[i], OUTPUT);
    digitalWrite(ENC_pin[i], HIGH);
  }

  SHOULDER_STEPPER.setMaxSpeed(8000);
  SHOULDER_STEPPER.setAcceleration(1500.0);
  
  ELBOW_STEPPER.setMaxSpeed(8000);
  ELBOW_STEPPER.setAcceleration(1000.0);
  
  BASE_STEPPER.setMaxSpeed(500);
  BASE_STEPPER.setAcceleration(100.0);

  PINZA_STEPPER.setMaxSpeed(4000);
  PINZA_STEPPER.setAcceleration(500.0);
  //Get the CS line high which is the default inactive state


  SPI.setClockDivider(SPI_CLOCK_DIV32);    // 500 kHz

  //start SPI bus
  SPI.begin();

  //mqtt configuration

  client.setServer(server, 1883);
  client.setCallback(callback);
  
  //ethernet configuration
  Ethernet.init(10);  // Most Arduino shields
  //Ethernet.begin(mac, ip);
  ethernet_init();

  delay(1500);
}


void loop()
{
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

moveARMStepper(joint_speed_req[0],joint_speed_req[1],joint_speed_req[2]);
move_EndEffector(pinza_speed_req);

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
if(strcmp(topic,"ARM/JOINT/1") == 0){
  

  if(f < -0.2 || f > 0.2)
  joint_speed_req[0] = f;

  else
  joint_speed_req[0] = 0; 
}

if(strcmp(topic,"ARM/JOINT/2") == 0){
  

  if(f < -0.2 || f > 0.2)
  joint_speed_req[1] = f;

  else
  joint_speed_req[1] = 0; 
}

if(strcmp(topic,"ARM/JOINT/3") == 0){
  

  if(f < -0.2 || f > 0.2)
  joint_speed_req[2] = f;

  else
  joint_speed_req[2] = 0; 
}

if(strcmp(topic,"ARM/EE") == 0){
  Serial.println("End effector req recevied");
  if(f < -0.3 || f > 0.3)
  pinza_speed_req = f;

  else
  pinza_speed_req = 0; 
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
      
      client.publish("ARM_heartbeat", "hello world");
      // ... and resubscribe

      //subscribe to all useful topic
      client.subscribe("ARM/JOINT/1");
      client.subscribe("ARM/JOINT/2");
      client.subscribe("ARM/JOINT/3");
      client.subscribe("ARM/EE");



      
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void getARM_joint_pos() {
   uint16_t encoderPosition;
  //let's also create a variable where we can count how many times we've tried to obtain the position in case there are errors
 int i;
 
  for (i = 0; i < 3; i++) {
    joint_pos_act[i] = getPositionSPI(ENC_pin[i], RES12);
    Serial.print("Encoder ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(joint_pos_act[i]);
    Serial.print(" - ");

  }

  Serial.println("\n");
}


void moveARMStepper(float speed_q1, float speed_q2, float speed_q3) {

  BASE_STEPPER.setSpeed(speed_q3*1000); //SI APRE
  SHOULDER_STEPPER.setSpeed(-speed_q2*2000);
  ELBOW_STEPPER.setSpeed(speed_q1*2000); //SI APRE

  BASE_STEPPER.runSpeed(); //SI APRE
  SHOULDER_STEPPER.runSpeed();
  ELBOW_STEPPER.runSpeed(); //SI APRE
  
}

void move_EndEffector(float speed_ee) {
    PINZA_STEPPER.setSpeed(speed_ee*500); //SI APRE
    PINZA_STEPPER.runSpeed(); //SI APRE
}


int degToSTEP(float input) {
  //la risoluzione è 1.8gradi (200 a rivoluzione)
  int stepNum = (input * 500) / (1.8 / 1600); //1600microstep + riduzione 75:1 + 3:1
  return stepNum;

}

//void disable_stepper {
//      digitalWrite(BASE_EN, HIGH);
//      digitalWrite(SHOULDER_EN, HIGH);
//      digitalWrite(ELBOW_EN, HIGH);
//      digitalWrite(PINZA_EN, HIGH);
//}
