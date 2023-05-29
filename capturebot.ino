#include <Servo.h>

Servo servo1;
Servo servo2;
int faceX = 480;
int faceY = 270;

void setup() {
  Serial.begin(9600);
  servo1.attach(9);
  servo2.attach(10);

  // Set the initial position of the servos
  servo1.write(90);
  servo2.write(40);
}

void receivePosition() {
  // Wait for data to be available
  while (!Serial.available()) {}
  // Read the X position
  String xString = Serial.readStringUntil(',');
  faceX = xString.toInt();
  if (faceX<0 or faceX>960){
    faceX = 480;
  }
  // Read the Y position
  String yString = Serial.readStringUntil('\n');
  faceY = yString.toInt();
  if (faceY<0 or faceY>540){
    faceY = 270;
  }
//  Serial.println("----------");
//  Serial.print("Received position: ");
//  Serial.print(faceX);
//  Serial.print(",");
//  Serial.println(faceY);
}

void moveServos() {
  // Map the X and Y positions to servo angles
  int servo1Tempt = map(faceX, 0, 960, 30, 150);
  int servo1Target = 180 - servo1Tempt;
  int servo2Target = map(faceY, 0, 540, 0, 110);
  // Gradually adjust the servo angles towards the new angles
  int servo1Current = servo1.read();
  int servo2Current = servo2.read();
  int servo1gap = (servo1Target - servo1Current);
  int servo2gap = (servo2Target - servo2Current);

  if (abs(servo1gap)>10){
    servo1Current += servo1gap/3;
    servo1.write(servo1Current);
    }

  if (abs(servo2gap)>10){
    servo2Current += servo2gap/3;
    servo2.write(servo2Current);
  }

//  Serial.println("----------");
//  Serial.print("Set servo angles: ");
//  Serial.print(servo1Target);
//  Serial.print(",");
//  Serial.println(servo2Target);
}

void loop() {
  receivePosition();
  //delay(10); // Add a small delay to allow servo motors to move to the correct position
  moveServos();
  delay(10);
}
