const int relayPin = 8;
bool relay = false;
bool manual = false;
bool autonomous = false;


void setup() {
    pinMode(relayPin, OUTPUT);
    Serial.begin(9600);
    digitalWrite(relayPin , HIGH);
}

void ModeSelector(int value){ 

  if(value==109){
    manual = true;
    autonomous = false;
  }
  else if(value==117){
    autonomous = true;
    manual = false;
  }
  else{
    autonomous = false;
    manual = false;

  }

}

void relaycontrol(int value) {
    if (value == 0) {
        relay = true;
        // Turn relay OFF
    } 
    else if (value == 1) {
        relay = false;   // Turn relay ON
    }
    digitalWrite(relayPin, relay ? HIGH : LOW);
}

void loop() {
    if (Serial.available() > 0) {
        int arr[2];  // Array to store received values
        int index = 0;
        String input = "";

        // Read input until newline character
        while (Serial.available() > 0) {
            char c = Serial.read();
            if (c == '\n') break;  // Stop at newline
            input += c;
            delay(2); // Small delay for complete transmission
        }

        // Parse the received string
        int firstComma = input.indexOf(',');
        if (firstComma > 0) {
            arr[0] = input.substring(0, firstComma).toInt();  // Arm Flag
            arr[1] = input.substring(firstComma + 1).toInt(); // Mode FLag
            
            relaycontrol(arr[0]);  // Control relay based on the first value
            ModeSelector(arr[1]);
            Serial.println(arr[0]);
        } 
    }
}
