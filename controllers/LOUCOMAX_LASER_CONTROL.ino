// Constants
const int analogPin = A0; // Pin A0 for analog input
#define LASER_CONTROL_PIN 2


void setup() {
  // Initialize the serial communication at a baud rate of 9600:
  Serial.begin(9600);

  // Initialize the LASER ON/OFF command pin
  pinMode(LASER_CONTROL_PIN, OUTPUT);
  digitalWrite(LASER_CONTROL_PIN, HIGH); // Laser OFF au démarrage
}

void loop() {
  // Read the analog input on pin A0:
  int sensorValue = analogRead(analogPin);
  
  // Print the sensor value to the Serial Monitor:
  Serial.println(sensorValue);
  
  // Delay in milliseconds before next reading:
  delay(10);

  // Laser On/Off command from serial com
  if (Serial.available()) {
    char command = Serial.read();
    if (command == '1') {
      digitalWrite(LASER_CONTROL_PIN, HIGH); // Laser ON
    } else if (command == '0') {
      digitalWrite(LASER_CONTROL_PIN, LOW);  // Laser OFF
    }
  }
}
