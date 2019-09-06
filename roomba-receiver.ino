//Receiver program
#include <SPI.h>
#include <SoftwareSerial.h>
#include "RF24.h"

#ifndef RF24_MAX_PAYLOAD_SIZE
// nrf24.h does not declare a maximum payload size
#define RF24_MAX_PAYLOAD_SIZE 32
#endif //RF24_MAX_PAYLOAD_SIZE

#define MASK_SIZE        0x1f //0b00011111 // max 31 bytes of data
#define MASK_WAKE_UP     0x20 //0b00100000
#define MASK_CHANGE_BAUD 0x40 //0b01000000
#define MASK_MORE_DATA   0x80 //0b10000000 // if true, load another message into buffer to complete data

#define BAUDE_PIN 7

const byte RF_ADDRESS_CONTROL[5] = {'A','B','C','D','E'};
const byte RF_ADDRESS_ROOMBA[5] = {'F','G','H','I','J'};

unsigned char in_val[RF24_MAX_PAYLOAD_SIZE];
unsigned char out_val[SERIAL_TX_BUFFER_SIZE];
unsigned long t = millis();

RF24 radio(10, 9);

void wake_up_roomba()
{
    digitalWrite(BAUDE_PIN, LOW);
    digitalWrite(BAUDE_PIN, HIGH);
}

void roomba_error() {
  unsigned char start[] = {128, 131};
  Serial.write(start, sizeof(start));
  delay(100);
  unsigned char song[] = {140, 1, 7, 40, 8, 64, 8, 40, 8, 52, 8, 40, 8, 52, 8, 64, 8, 141, 1};
  Serial.write((unsigned char *)&song[0], sizeof(song));
  delay(1000);
  unsigned char done[] = {128};
  Serial.write(done, sizeof(done));
}

uint8_t checksum(char *data, uint16_t len) {
    uint8_t crc = 0;
    for (int i = 0; i<len; i++) {
        crc += data[i];
    }
    crc ^= 0xff;
    return crc;
}

void setup()
{
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  pinMode(BAUDE_PIN, OUTPUT);
  digitalWrite(BAUDE_PIN, HIGH);

  radio.begin();
  //radio.setRetries(10, 5);
  radio.enableDynamicPayloads();
  radio.setDataRate( RF24_2MBPS );
  radio.setChannel(108); //
  radio.openWritingPipe(RF_ADDRESS_CONTROL);
  radio.openReadingPipe(1,RF_ADDRESS_ROOMBA);
  radio.startListening();

  roomba_error();
}

void loop()
{
  if (radio.available()) { //When the program is received, the received data is output from the serial port
    char data_size;
    int crc;
    uint8_t payload_size = radio.getDynamicPayloadSize();

    radio.read(&in_val, payload_size);

    // check 1st bit for signals and data size
    if ( in_val[0] & MASK_WAKE_UP ) {
      wake_up_roomba();
    }
    data_size = in_val[0] & MASK_SIZE;

    if ( !checksum(in_val, payload_size) ) {
      Serial.write( (unsigned char *)&in_val[1], data_size );
    } else {
      roomba_error();
    }
  }


// roomba sends to serial every 15ms, that is too much, only read every Xms
  if ( Serial.available() & ((millis()-t) > 250) ) { 
    radio.openWritingPipe(RF_ADDRESS_ROOMBA);
    radio.openReadingPipe(1,RF_ADDRESS_CONTROL);
    radio.stopListening();

    int sz_avail = Serial.available();
    while (sz_avail > 0) {
      int sz_payload = (sz_avail > RF24_MAX_PAYLOAD_SIZE) ? RF24_MAX_PAYLOAD_SIZE : sz_avail;
      Serial.readBytes((char *)&out_val, sz_payload);
      radio.write(&out_val, sz_payload);
      sz_avail -= sz_payload;
    }
    radio.openWritingPipe(RF_ADDRESS_CONTROL);
    radio.openReadingPipe(1,RF_ADDRESS_ROOMBA);
    radio.startListening();
    t = millis();
  }
}
