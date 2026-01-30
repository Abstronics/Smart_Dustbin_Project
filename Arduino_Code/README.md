#include <Servo.h>

Servo lidServo;

// Ultrasonic pins
const int trigPin = 9;
const int echoPin = 10;

// Servo pin
const int servoPin = 6;

// Variables
long duration;
int distance;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  lidServo.attach(servoPin);
  lidServo.write(0);   // Lid closed

  Serial.begin(9600);
}

void loop() {
  // Clear trig
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Send ultrasonic pulse
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read echo
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;

  Serial.print("Distance: ");
  Serial.println(distance);

  // If hand/object is close
  if (distance > 0 && distance <= 15) {
    lidServo.write(90);   // Open lid
    delay(3000);          // Keep open for 3 seconds
    lidServo.write(0);    // Close lid
  }

  delay(200);
}
