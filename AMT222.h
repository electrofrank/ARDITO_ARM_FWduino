#include "Arduino.h"

#ifndef INC_AMT222_H
#define INC_AMT222_H

/* SPI commands */
#define AMT22_NOP       0x00
#define AMT22_RESET     0x60
#define AMT22_ZERO      0x70

/* Define special ascii characters */
#define NEWLINE         0x0A
#define TAB             0x09

/* We will use these define macros so we can write code once compatible with 12 or 14 bit encoders */
#define RES12           12

/* SPI pins */
#define ENC_0           48
#define ENC_1           53
#define SPI_MOSI        51
#define SPI_MISO        50
#define SPI_SCLK        52

uint16_t getPositionSPI(uint8_t encoder, uint8_t resolution);

uint8_t spiWriteRead(uint8_t sendByte, uint8_t encoder, uint8_t releaseLine);

void setCSLine (uint8_t encoder, uint8_t csLine);

void setZeroSPI(uint8_t encoder);

void resetAMT22(uint8_t encoder);


  #endif /* INC_AMT222_H */
