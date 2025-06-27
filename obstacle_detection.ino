/*
  Ultrasonic Sensor HC-SR04 and Arduino Tutorial

  by Dejan Nedelkovski,
  www.HowToMechatronics.com

*/
// defines pins numbers
const int trigPin = 9;
const int echoPin = 10;
// defines variables
long duration;
int distance;
int checkingInterval = 1000; //in ms
int triesBeforeObstacle = 2; // number of consucutive detections that must happen before we consider something in the front an obstacle
int consucutiveDetections = 0;
int detectionThreshold = 50; // in cm

void processDetection(int distance){

  // Prints the distance on the Serial Monitor
  // Serial.print("Processing Distance: ");
  // Serial.println(distance);

  if (distance <= 50) {
    if (consucutiveDetections == 0) {
      Serial.println("stop");
    }
    consucutiveDetections++;
  } else {
    if (consucutiveDetections != 0 && consucutiveDetections < triesBeforeObstacle) { // we send continue order after a stop order, and before declaring an obstacle, if an obstacle is detected, the user will continue directly after rerounting, no need to send a continue order
      Serial.println("continue");
    }
    consucutiveDetections = 0;
  }

  if (consucutiveDetections == triesBeforeObstacle) {
    // declare an obstacle
    Serial.print("obstacle:");
    Serial.println(distance);
  }

}


void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600); // Starts the serial communication
}
void loop() {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;

  processDetection(distance);
  
  delay(checkingInterval);
}
