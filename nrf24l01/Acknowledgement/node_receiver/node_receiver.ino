/*
   See documentation at https://nRF24.github.io/RF24
   See License information at root directory of this library
   Author: Brendan Doherty (2bndy5)
*/

/**
   A simple example of sending data from 1 nRF24L01 transceiver to another
   with Acknowledgement (ACK) payloads attached to ACK packets.

   This example was written to be used on 2 devices acting as "nodes".
   Use the Serial Monitor to change each node's behavior.
*/
#include <SPI.h>
#include "printf.h"
#include "RF24.h"

// instantiate an object for the nRF24L01 transceiver
RF24 radio(7, 8); // using pin 7 for the CE pin, and pin 8 for the CSN pin

// an identifying device destination
// Let these addresses be used for the pair

uint8_t ReadingPipeAddress[6] = "1Node";
uint8_t WritingPipeAddress[6] = "2Node";
// It is very helpful to think of an address as a path instead of as
// an identifying device destination
// to use different addresses on a pair of radios, we need a variable to


// For this example, we'll be using a payload containing
// a string & an integer number that will be incremented
// on every successful transmission.
// Make a data structure to store the entire payload of different datatypes
struct PayloadStruct {
  char message[7];          // only using 6 characters for TX & ACK payloads
  uint8_t counter;
};
PayloadStruct payload;

void setup() {

  Serial.begin(115200);
  while (!Serial) {
    // some boards need to wait to ensure access to serial over USB
  }

  // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {} // hold in infinite loop
  }

  // print example's introductory prompt
  Serial.println(F("receiver node"));


  // Set the PA Level low to try preventing power supply related problems
  // because these examples are likely run with nodes in close proximity to
  // each other.
  radio.setPALevel(RF24_PA_LOW);     // RF24_PA_MAX is default.

  // to use ACK payloads, we need to enable dynamic payload lengths (for all nodes)
  radio.enableDynamicPayloads();    // ACK payloads are dynamically sized

  // Acknowledgement packets have no payloads by default. We need to enable
  // this feature for all nodes (TX & RX) to use ACK payloads.
  radio.enableAckPayload();

  // set the TX address of the RX node into the TX pipe
  radio.openWritingPipe(WritingPipeAddress);     // always uses pipe 0

  // set the RX address of the TX node into a RX pipe
  radio.openReadingPipe(1, ReadingPipeAddress); // using pipe 1


  // setup the ACK payload & load the first response into the FIFO
  memcpy(payload.message, "World ", 6);                       // set the payload message
  // load the payload for the first received transmission on pipe 0
  radio.writeAckPayload(1, &payload, sizeof(payload));

  radio.startListening();                                     // put radio in RX mode

  // For debugging info
  // printf_begin();             // needed only once for printing details
  // radio.printDetails();       // (smaller) function that prints raw register values
  // radio.printPrettyDetails(); // (larger) function that prints human readable data

}

void loop() {



  uint8_t pipe;
  if (radio.available(&pipe)) {                    // is there a payload? get the pipe number that recieved it
    uint8_t bytes = radio.getDynamicPayloadSize(); // get the size of the payload
    PayloadStruct received;
    radio.read(&received, sizeof(received));       // get incoming payload
    Serial.print(F("Received "));
    Serial.print(bytes);                           // print the size of the payload
    Serial.print(F(" bytes on pipe "));
    Serial.print(pipe);                            // print the pipe number
    Serial.print(F(": "));
    Serial.print(received.message);                // print incoming message
    Serial.print(received.counter);                // print incoming counter
    Serial.print(F(" Sent: "));
    Serial.print(payload.message);                 // print outgoing message
    Serial.println(payload.counter);               // print outgoing counter

    // save incoming counter & increment for next outgoing
    payload.counter = received.counter + 1;
    // load the payload for the first received transmission on pipe 0
    radio.writeAckPayload(1, &payload, sizeof(payload));
  }



} // loop
