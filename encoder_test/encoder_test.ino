#include <AMT222.h>
#include <SPI.h>

/* SPI pins */

int ENC_pin[3];
int i;


void setup()
{

  //Set the modes for the SPI IO
  pinMode(SPI_SCLK, OUTPUT);
  pinMode(SPI_MOSI, OUTPUT);
  pinMode(SPI_MISO, INPUT);

  ENC_pin[0] = 44;
  ENC_pin[1] = 40;
  ENC_pin[2] = 36;

  for (i = 0; i < 3; i++) {
    pinMode(ENC_pin[i], OUTPUT);
    digitalWrite(ENC_pin[i], HIGH);
  }
  //Initialize the UART serial connection for debugging
  Serial.begin(115200);
  SPI.setClockDivider(SPI_CLOCK_DIV32);    // 500 kHz

  //start SPI bus
  SPI.begin();
}

void loop()
{
  //create a 16 bit variable to hold the encoders position
  uint16_t encoderPosition;
  //let's also create a variable where we can count how many times we've tried to obtain the position in case there are errors
  uint8_t attempts;

  float joint_pos_act[3]; //actual joint position, degree

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
