/*
 * 
 * Run this sketch to display numbers 1-5 on the LED displays 
 * To identify which Serial contorls which display
 * 
 */

#include <ArduinoJson.h>
#include <SoftwareSerial.h>

SoftwareSerial qSerial5(21, 20);
SoftwareSerial qSerial4(2, 3);

#define MAX_HEX_MESSAGE 1024

// Buffers for the messages
byte display_1_Hex[MAX_HEX_MESSAGE];
byte display_2_Hex[MAX_HEX_MESSAGE];
byte display_3_Hex[MAX_HEX_MESSAGE];
byte display_4_Hex[MAX_HEX_MESSAGE];
byte display_5_Hex[MAX_HEX_MESSAGE];

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);      // Debugging via Serial Monitor
  // Front
  Serial1.begin(115200);     // Communication with LED panel
  // Rear
  Serial2.begin(115200);     // Communication with LED panel
  // Left
  Serial3.begin(115200);     // Communication with LED panel
  // Right
  qSerial4.begin(115200);    // SDA/SCL Port
  //
  qSerial5.begin(115200);


  

}

void loop() {
  // put your main code here, to run repeatedly:

  if (Serial.available() > 0){

    String input1 = Serial.readStringUntil('\n');  // Read the input until a newline
    Serial.println("Received: " + input1 + "\n");
    delay(10);
    
  }

  int rgb[] = {255, 255, 255};

  int display_1_HexLength = printText("FRONT", rgb, 20, display_1_Hex); // Front
  int display_2_HexLength = printText("REAR", rgb, 20, display_2_Hex); //
  int display_3_HexLength = printText("LEFT", rgb, 20, display_3_Hex); // Left
  int display_4_HexLength = printText("RIGHT", rgb, 20, display_4_Hex); // Right
  int display_5_HexLength = printText("TOP", rgb, 20, display_5_Hex);

  Serial1.write(display_1_Hex, display_1_HexLength);
  Serial2.write(display_2_Hex, display_2_HexLength);
  Serial3.write(display_3_Hex, display_3_HexLength);
  qSerial4.write(display_4_Hex, display_4_HexLength);
  qSerial5.write(display_5_Hex, display_5_HexLength);

}





// Returns hexmessage for printing text
int printText (String message, int rgb[], int duration, byte* hexMessage) {

  int messageLength = message.length();
  byte messageinHex[messageLength];
  stringToByteArray(message, messageinHex, messageLength);

  String input = "A5 68 32 01 7B 01 13 00 00 00 11 03 0b 01 00 00 03 02 00 12 ff 4D 41 52 59 00 a7 04 AE";
  int byteArraySize = countHexValues(input);
  byte initHexMessage[byteArraySize];
  convertToByteArray(input, initHexMessage, byteArraySize); // Converts input into hexMessage (byteArray)

  int length_ = sizeof(initHexMessage) / sizeof(initHexMessage[0]) - 3;   // Length of initHexMessage minus the ending code and checksum
  int length_array_ = sizeof(initHexMessage) / sizeof(initHexMessage[0]); // Length of the entire hexMessage

  int startBytesLength = 21;
  int endBytesLength = 4;

  byte startBytes[startBytesLength]; // For bytes upto the text
  byte endBytes[endBytesLength];     // For bytes after the text

  int hexMessageLength = startBytesLength + messageLength + endBytesLength;
  //byte hexMessage[hexMessageLength];

  // Copying required bytes from original message
  memcpy(startBytes, initHexMessage, 21);
  memcpy(endBytes, &initHexMessage[length_array_ - 4], 4);
 
  // Combining all to get new Hex Message
  memcpy(hexMessage, startBytes, startBytesLength);
  memcpy(&hexMessage[startBytesLength], messageinHex, messageLength);
  memcpy(&hexMessage[startBytesLength + messageLength], endBytes, endBytesLength);

//  int length = sizeof(hexMessage) / sizeof(hexMessage[0]) - 3;   // Length of hexMessage minus the ending code and checksum
//  int length_array = sizeof(hexMessage) / sizeof(hexMessage[0]); // Length of the entire hexMessage

  int length = hexMessageLength - 3;   // Length of hexMessage minus the ending code and checksum
  int length_array = hexMessageLength; // Length of the entire hexMessage

  hexMessage[18] = (byte)rgb[0];  // R
  hexMessage[19] = (byte)rgb[1];  // G
  hexMessage[20] = (byte)rgb[2];  // B

  int scrollLength = 5;
  
//  if (isLargeDisplay){
//    scrollLength = 8;
//  } else {
//    scrollLength = 4;
//  }
  
  if (messageLength > scrollLength){
    
    // Effect
    hexMessage[12] = 11; // Continuous scroll to left
    // Playtime - Number of times effect runs
    hexMessage[11] = 10;
    // Font
    // 0x12 for small text (Only on top half of screen - 7 Characters)
    // 0x14 for large text (Centered on screen - 4 Characters)
    // Always centered for scroll
    hexMessage[17] = 0x13; // 0xAB A is Font style code from document and B is Font size code from the document
    
  } else {

    // Effect
    hexMessage[12] = 59; // Static
    // Playtime - For static set to 1, then runs for the duration of stay time
    hexMessage[11] = 1;
    // Font
    // 0x12 for small text (Only on top half of screen - 7 Characters)
    // 0x14 for large text (Centered on screen - 4 Characters)
    hexMessage[17] = 0x14; // 0xAB A is Font style code from document and B is Font size code from the document
    
  }

  // Align
  hexMessage[13] = 0x05; // 0x04 - Left Aligned, 0x05 - Centered, 0x06 - Right Aligned

  // Stay time (15/16)
  byte time_lowByte = duration & 0xFF;         // Get the low byte (least significant byte)
  byte time_highByte = (duration >> 8) & 0xFF; // Get the high byte (most significant byte)
  hexMessage[15] = time_highByte;
  hexMessage[16] = time_lowByte;
  
  // Packet Length (6/7)
  int packet_length = length_array - 13;       // CC length
  byte lowByte = packet_length & 0xFF;         // Get the low byte (least significant byte)
  byte highByte = (packet_length >> 8) & 0xFF; // Get the high byte (most significant byte)
  hexMessage[6] = lowByte;
  hexMessage[7] = highByte;

  uint16_t checksum_lower;
  uint16_t checksum_higher;
  uint16_t checksum = calculateChecksum(hexMessage, length);
  checksum_lower = checksum % 256;  
  checksum_higher = checksum / 256;

  hexMessage[length_array-3] = checksum_lower;
  hexMessage[length_array-2] = checksum_higher;

//  Serial.println("New HexMessage : ");
//  printByteArray(hexMessage, sizeof(hexMessage));
//  delay(10);

  return hexMessageLength;

}

// Converts a string with hex values to a byte array
void convertToByteArray(String hexString, byte* byteArray, int arraySize) {
  int index = 0;
  char* token = strtok(const_cast<char*>(hexString.c_str()), " ");
  
  while (token != nullptr && index < arraySize) {
    byteArray[index] = strtol(token, nullptr, 16); // Convert hex string to byte
    token = strtok(nullptr, " ");
    index++;
  }
}


 
// Counts the number of values
int countHexValues(String hexString) {
  int count = 0;
  for (int i = 0; i < hexString.length(); i++) {
    if (hexString.charAt(i) == ' ') count++;
  }
  return count + 1; // Add 1 for the last value
}


 
// Function to calculate the 2-byte checksum
uint16_t calculateChecksum(byte data[], int length) {
  uint16_t checksum = 0;
 
  // Sum all the bytes in the array
  for (int i = 0; i < length; i++) {
    if(i>0 )
    checksum += data[i];
    // Serial.print(data[i]);
    // Serial.print(" - ");
    // Serial.println(checksum);
  }
 
  // Apply modulo 0x10000 to ensure it fits in 2 bytes
  checksum = checksum & 0xFFFF;
 
  return checksum;
}


 
// String to Hex
void stringToByteArray(String inputText, byte* byteArray, int arraySize) {
  for (int i = 0; i < inputText.length() && i < arraySize; i++) {
    byteArray[i] = (byte)inputText.charAt(i);  // Convert each character to a byte
  }
}



void printByteArray(byte arr[], int length) {
  for (int i = 0; i < length; i++) {
    delay(10);
    Serial.print("0x");
    if (arr[i] < 0x10) {
      Serial.print("0"); // Leading zero for single digits
    }
    Serial.print(arr[i], HEX);
    if (i < length - 1) {
      Serial.print(", ");
    }
  }
  Serial.println();
}
  
