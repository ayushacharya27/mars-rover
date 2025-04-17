int motor_forward1 = 11;
int motor_backward1 = 10;
int motor_forward2 = 9;
int motor_backward2 = 6;

void setup() {
  pinMode(motor_forward1, OUTPUT);
  pinMode(motor_backward1, OUTPUT);
  pinMode(motor_forward2, OUTPUT);
  pinMode(motor_backward2, OUTPUT);
  
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    int correction = Serial.parseInt(); // Read integer value sent from Python/ROS

    int speed = constrain(abs(correction), 0, 255);

    if (correction > 0) {
      // Rotate right
      analogWrite(motor_forward1, speed);
      analogWrite(motor_backward1, 0);
      analogWrite(motor_forward2, 0);
      analogWrite(motor_backward2, speed);
    }
    else if (correction < 0) {
      // Rotate left
      analogWrite(motor_forward1, 0);
      analogWrite(motor_backward1, speed);
      analogWrite(motor_forward2, speed);
      analogWrite(motor_backward2, 0);
    }
    else {
      // Stop
      analogWrite(motor_forward1, 0);
      analogWrite(motor_backward1, 0);
      analogWrite(motor_forward2, 0);
      analogWrite(motor_backward2, 0);
    }

    // Optional: print received value for debugging
    Serial.print("Received correction: ");
    Serial.println(correction);
  }
}
