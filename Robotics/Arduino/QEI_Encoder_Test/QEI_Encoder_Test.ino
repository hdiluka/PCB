#include <Encoder.h>
const int encoderPinA = 2;//pin for encoder's channel A.
const int encoderPinB = 3;//pin for encoder's channel B.

Encoder myEnc(encoderPinA, encoderPinB);//Initialize Encoder. (Quad Mode, Interrupt Enabled)
volatile long currentPosition = 0;

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
  Serial.println("Encoder Test");
}

void loop() {
  // put your main code here, to run repeatedly:
currentPosition = myEnc.read();
Serial.println(currentPosition);
}
