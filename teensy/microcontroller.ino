#include <_Teensy.h>
#include <Servo.h>

Servo servoSail;
Servo servoTail;

const int pwmPin3 = 3;
const int pwmPin4 = 4;
int incomingByte = 0;

String servoInput1 = "";  // For SerialUSB1 input
String servoInput2 = "";  // For SerialUSB2 input

const int anemometerPin = A0;  // ADC pin for the anemometer
float windAngle;

void setup() {
  Serial.begin(9600); //Anem
  Serial1.begin(9600); //Sail
  Serial2.begin(9600); //Tail
  servoTail.attach(4);
  servoSail.attach(3);
  pinMode(pwmPin3, OUTPUT);
  pinMode(pwmPin4, OUTPUT);
  servoSail.write(1050);
}

// Function to convert anemometer reading to wind angle
float calculateWindAngle(int sensorValue) {
  return map(sensorValue, 0, 1023, 0, 360);  // Map analog reading to 0-360 degrees
}

void loop() {
 // Non-blocking read for first servo (from SerialUSB1)
  if (SerialUSB1.available()) {
    char incomingByte = SerialUSB1.read();  // Read one byte
    if (incomingByte == '\n') {
      // End of command, convert input to angle and move servo1
      int sail_angle = servoInput1.toInt();
      servoSail.write(map(sail_angle, 0, 90, 1050, 1300));  // Move servo1
      SerialUSB1.print("Sail set to PWM: ");
      SerialUSB1.println(sail_angle);
      servoInput1 = "";  // Clear buffer after processing
    } else {
      servoInput1 += incomingByte;  // Accumulate characters
    }
  }

  // Non-blocking read for second servo (from SerialUSB2)
  if (SerialUSB2.available()) {
    char incomingByte = SerialUSB2.read();  // Read one byte
    if (incomingByte == '\n') {
      // End of command, convert input to angle and move servo1
      int tail_angle = servoInput2.toInt();
      servoTail.write(map(tail_angle, -30, 30, 1300, 1690));  // Move servo1
      SerialUSB2.print("Tail set to PWM: ");
      SerialUSB2.println(tail_angle);
      servoInput2 = "";  // Clear buffer after processing
    } else {
      servoInput2 += incomingByte;  // Accumulate characters
    }
  }
  delay(50); 
 }
}
