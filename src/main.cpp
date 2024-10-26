
#include <Arduino.h>
//#include <SoftwareSerial.h>
#include <LiquidCrystal.h>

//SoftwareSerial mySerial(6, 7); // RX, TX for MAX3232 communication

const int rs = A3, en =A2, d4 = 13, d5 = 12, d6 = 11, d7 = 10;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

char t[32];



//-------------------------LIPO4 Functions-----------------------------------------------
void pipSend(unsigned char *cmd, int len);
uint16_t crc16(const uint8_t* data, uint8_t length);
String getValue(String data, char separator, int index);
bool matchesFormat(const String& value, const String& expected);
bool isValidResponse(const String& data);



// Some useful PIP commands
struct pipCommands_t {
    unsigned char qpigs[5];
} pipCommands = {'Q', 'P', 'I', 'G', 'S'};

void setup() {
    Serial.begin(2400); // Serial Monitor Console Baud Setting
    //mySerial.begin(2400);  // Start serial communication with the inverter at 2400 baud


//---------------------lcd
pinMode(9,OUTPUT);
digitalWrite(9,1);
lcd.begin(16,2);
lcd.clear();
lcd.noCursor();
lcd.setCursor(0,0);
lcd.print(" SLC LiPo4 V3.0 ");
lcd.setCursor(0,1);
lcd.print("    KATELEC ");
delay(1500);
lcd.clear();
}

void loop() {
    // Send command to inverter
    pipSend(pipCommands.qpigs, sizeof(pipCommands.qpigs));

    // Read and print the incoming serial data from the inverter until <CR>
    String receivedData = "";
    bool endOfResponse = false;  // Flag to indicate when end of response is found

    while (!endOfResponse) {
        if (Serial.available()) {
            char incomingByte = Serial.read();
            receivedData += incomingByte;

            // Check for carriage return '\r'
            if (incomingByte == '\r') {
                endOfResponse = true;  // Stop reading when <CR> is found
            }
        }
    }

    // Print the received data
   // Serial.print("Received Data: ");
    //Serial.println(receivedData);

    // Check if the response matches the expected format
    if (isValidResponse(receivedData)) {
       // Serial.println("Response is accepted.");

        // Extract and print battery voltage
        String batteryVoltage = getValue(receivedData, ' ', 8);  // 9th item (index 8)
        //Serial.print("Battery Voltage: ");
        //Serial.println(batteryVoltage);
 

        // Extract the battery capacity (10th index, 0-based)
        String batteryCapacity = getValue(receivedData, ' ', 10); // 11th item (index 10)
       // Serial.print("Battery Capacity: ");
      //  Serial.println(batteryCapacity+'%');
        lcd.setCursor(0,1);
        lcd.print(batteryCapacity+'%');
      //  Serial.print("%");
    } else {
       // Serial.println("Response is NOT accepted.");
    }

   // delay(100);  // Wait for 5 seconds before sending the next command
}

// Function to validate the received response format
bool isValidResponse(const String& data) {
    // Ensure the data is long enough to have all expected parts
    if (data.length() < 40) {
        return false;
    }
  
    

    // Remove the starting '(' and ending character for processing
    String content = data.substring(1, data.length() - 1);
    int startIndex = 0;
    int spaceIndex;

    // Validate each expected part of the response
    String expectedFormats[] = {
        "000.0", // BBB.B
        "00.0",  // CC.C
        "000.0", // DDD.D
        "00.0",  // EE.E
        "0000",  // FFFF
        "0000",  // GGGG
        "000",   // HHH
        "339",   // III
        "25.60", // JJ.JJ
        "000",   // KKK
        "100",   // OOO
        "0030"   // TTTT
    };

    for (int i = 0; i < sizeof(expectedFormats) / sizeof(expectedFormats[0]); i++) {
        spaceIndex = content.indexOf(' ', startIndex);
        String segment;

        // If no space is found, this is the last segment
        if (spaceIndex == -1) {
            segment = content.substring(startIndex);
        } else {
            segment = content.substring(startIndex, spaceIndex);
        }

        // Check the segment against the expected format
        if (!matchesFormat(segment, expectedFormats[i])) {
            return false; // Format mismatch
        }

        // Move startIndex to the next part
        startIndex = spaceIndex + 1;
    }

    return true; // All parts match the expected format
}

// Function to check if a segment matches the expected format
bool matchesFormat(const String& value, const String& expected) {
    if (expected.length() != value.length()) return false;

    for (int i = 0; i < expected.length(); i++) {
        char expChar = expected.charAt(i);
        char valChar = value.charAt(i);

        if (expChar == '0') {
            if (!isDigit(valChar)) return false; // Must be a digit
        } else if (expChar == '.') {
            if (valChar != '.') return false; // Must be a dot
        }
    }
  
    return true; // Matches the expected format
}

// Helper function to split and get the Nth value from the response
String getValue(String data, char separator, int index) {
    int found = 0;
    int strIndex = 0; // Current index in the string
    int startIndex = 0; // Starting index for the current value

    // Loop through the data to find the index
    while (strIndex < data.length()) {
        // If we find the separator or reach the end of the string
        if (data.charAt(strIndex) == separator || strIndex == data.length() - 1) {
            found++;
            // If it's the last character, adjust the end index
            int endIndex = (strIndex == data.length() - 1) ? strIndex + 1 : strIndex;
            // Check if this is the index we want
            if (found - 1 == index) { // -1 because we're counting from 0
                // Create a substring and trim it
                String value = data.substring(startIndex, endIndex);
                value.trim(); // Trim to remove extra spaces
                return value; // Return the trimmed value
            }
            // Update the startIndex for the next value
            startIndex = strIndex + 1;
        }
        strIndex++;
    }

    return ""; // Return empty string if not found
}

// CRC-16/IBM Calculation
uint16_t crc16(const uint8_t* data, uint8_t length) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }
    return crc;
}

void pipSend(unsigned char *cmd, int len) {
    uint16_t crc = crc16(cmd, len);  // Compute CRC

    // Send command to inverter
    for (int i = 0; i < len; i++) {
        Serial.write(cmd[i]);
    }

    // Send CRC and carriage return
    Serial.write(crc & 0xFF);  // CRC low byte
    Serial.write(crc >> 8);    // CRC high byte
    Serial.write(0x0d);        // Carriage return
}
