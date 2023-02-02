/*
  Fading

  This example shows how to fade an LED using the analogWrite() function.

  The circuit:
  - LED attached from digital pin 9 to ground through 220 ohm resistor.

  created 1 Nov 2008
  by David A. Mellis
  modified 30 Aug 2011
  by Tom Igoe

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Fading
*/

#define RED 18   // LED connected to digital pin 9
#define GREEN 19
#define BLUE 20

#define delayTime 5

byte redVal=255;

byte greenVal=0;

byte blueVal=0;

void setup() {
  // nothing happens in setup
}

void loop() {
for( byte i = 0 ; i < 255 ; i++ ){

redVal--;

greenVal++;

analogWrite( RED, 255-redVal );

analogWrite( GREEN, 255-greenVal );

delay( delayTime );

}

for( byte i = 0 ; i < 255 ; i++ ){

greenVal--;

blueVal++;

analogWrite( GREEN, 255-greenVal );

analogWrite( BLUE, 255-blueVal );

delay( delayTime );

}

for( byte i = 0 ; i < 255 ; i++ ){

blueVal--;

redVal++;

analogWrite( BLUE, 255-blueVal );

analogWrite( RED, 255-redVal );

delay( delayTime );

}
}
