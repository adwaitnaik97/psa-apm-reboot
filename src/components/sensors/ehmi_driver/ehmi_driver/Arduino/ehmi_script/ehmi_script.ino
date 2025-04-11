#include <ArduinoJson.h>
#include <SoftwareSerial.h>


/*
 * Large Display --> 192x32
 * Small Display --> 64x32
 */

// Struct to store values received from Python
struct MessageData {
  String largeDisplay;
  String smallDisplay;
  int rgb[3];
  int duration; 
};

MessageData data;
String previousMsgSmall = ""; // To monitor when the text changes
String previousMsgLarge = ""; // To monitor when the text changes

// Right Arrow
byte rightArrow[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,    
  0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, // Row 1
  0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x00, 0x00, // Row 2
  0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, // Row 3
  0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, // Row 4
  0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, // Row 5
  0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, // Row 6
  0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, // Row 7
  0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, // Row 8
  0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, // Row 9
  0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, // Row 10
  0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, // Row 11
  0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, // Row 12
  0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x00, 0x00, // Row 13
  0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, // Row 14
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };

// Left Arrow
byte leftArrow[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, // Row 1
  0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, // Row 2
  0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, // Row 3
  0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, // Row 4
  0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, // Row 5
  0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, // Row 6
  0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0x00, 0x00, // Row 7
  0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0x00, 0x00, // Row 8
  0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, // Row 9
  0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, // Row 10
  0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, // Row 11
  0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, // Row 12
  0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, // Row 13
  0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00,  // Row 14
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

#define MAX_HEX_MESSAGE 512 // 768

// Buffers for the messages
byte largeDisplayHex[MAX_HEX_MESSAGE];
byte smallDisplayHex[MAX_HEX_MESSAGE];
byte leftArrowHex[MAX_HEX_MESSAGE];
byte rightArrowHex[MAX_HEX_MESSAGE];
byte solidColorHex[MAX_HEX_MESSAGE];
byte textGW_Hex[MAX_HEX_MESSAGE];

// 8426 & 8427
HardwareSerial &front = Serial1;
HardwareSerial &rear = Serial2; // Unconfirmed
HardwareSerial &left = Serial3; 
SoftwareSerial right(2, 3); 
SoftwareSerial top(21, 20);

// 8427
// HardwareSerial &front = Serial1;
// HardwareSerial &left = Serial3;
// HardwareSerial &right = Serial2;
// SoftwareSerial top(21, 20); 


void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);      // Debugging via Serial Monitor
  front.begin(115200);     // Communication with LED panel
  right.begin(115200);     // Communication with LED panel
  left.begin(115200);     // Communication with LED panel
  top.begin(115200);    // SDA/SCL Port

  setDisplayWindowSmall();
  setDisplayWindowBig();

}

void loop() {
  // put your main code here, to run repeatedly:

  if (Serial.available()){

    String input1 = Serial.readStringUntil('\n');  // Read the input until a newline
    input1.trim();

    if (input1.length() > 0) {

      Serial.println("Received: " + input1 + "\n");
      // delay(10);
    
    }
   
    // Parse JSON
    StaticJsonDocument<256> doc;  // Allocate memory for parsing
    DeserializationError error = deserializeJson(doc, input1);
   
    if (error) {
      
      Serial.println("JSON parsing failed!");
      
    } else {
      
      // Access the data and assign to the struct
      data.largeDisplay = doc[0].as<String>();
      data.smallDisplay = doc[1].as<String>();
      JsonArray rgb = doc[2];
      data.duration = doc[3];
  
      // Assign RGB values to the struct
      for (int j = 0; j < 3; j++) {
        data.rgb[j] = rgb[j];
      }
  
    }

    // Monitoring change in text 
    if (previousMsgSmall != data.smallDisplay){

      setDisplayWindowSmall();
      previousMsgSmall = data.smallDisplay;
      
    }

    if (previousMsgLarge != data.largeDisplay){

      setDisplayWindowBig();
      previousMsgLarge = data.largeDisplay;
      
    }

    // Clear screens
    if (data.largeDisplay == "" && data.smallDisplay == ""){

      setDisplayWindowSmall();
      setDisplayWindowBig();
      
    }
    
    // For Front & Rear
    int largeDisplayHexLength = printText(data.largeDisplay, data.rgb, true, largeDisplayHex);
    // When the text is Filter Left/Right this display needs to alternate between that and GIVE WAY (white)

    int smallDisplayHexLength = 0;
    int leftArrowHexLength = 0;
    int rightArrowHexLength = 0;
    int solidColorHexLength = 0;

    if (data.smallDisplay == "<-- -->" || data.smallDisplay == "--> <--"){

      leftArrowHexLength = printArrow(false, data.duration, leftArrowHex);
      rightArrowHexLength = printArrow(true, data.duration, rightArrowHex);
      
    }else if (data.smallDisplay == "Solid") {
      
      solidColorHexLength = printSolidColor(data.duration, data.rgb, solidColorHex);
      
    }else {
      
      smallDisplayHexLength = printText(data.smallDisplay, data.rgb, false, smallDisplayHex);
      
    }

    // Front and Rear
    // When the text is Filter Left/Right this display needs to alternate between that and GIVE WAY (white)
    front.write(largeDisplayHex, largeDisplayHexLength);
    //Serial2.write(largeDisplayHex, largeDisplayHexLength);

    // Left and Right
    if (data.smallDisplay == "<-- -->"){

      //setDisplayWindowSmall();
      left.write(leftArrowHex, leftArrowHexLength);
      right.write(rightArrowHex, rightArrowHexLength);
      
    } else if (data.smallDisplay == "--> <--"){

      //setDisplayWindowSmall();
      left.write(rightArrowHex, rightArrowHexLength);
      right.write(leftArrowHex, leftArrowHexLength);
      
    } else if (data.smallDisplay == "Solid"){

      //setDisplayWindowSmall(); // Doing this makes it blink (sort of)
      top.write(solidColorHex, solidColorHexLength);
      
    } else {

      //setDisplayWindowSmall();
      left.write(smallDisplayHex, smallDisplayHexLength);
      right.write(smallDisplayHex, smallDisplayHexLength);
      
    }
    
  } // Serial
  
} // Loop





/* 

Functions 

*/

// Ensures entire display is useable
void setDisplayWindowSmall(){

  // cc = 01 01 00 00 00 00 00 40 00 20
  String input = "A5 68 32 01 7B 01 0A 00 00 00 01 01 00 00 00 00 00 40 00 20 00 00 AE";

  // Count number of values
  int byteArraySize = countHexValues(input);

  // Create byte array with size of input
  byte hexMessage[byteArraySize];
  
  // Convert input into hexMessage (byteArray)
  convertToByteArray(input, hexMessage, byteArraySize); 

  //20/21 Checksum
  // Calculate checksum
  int length = byteArraySize - 3; // Length excluding end bytes
  uint16_t checksum = calculateChecksum(hexMessage, length);
  hexMessage[byteArraySize - 3] = checksum & 0xFF;        // Low byte
  hexMessage[byteArraySize - 2] = (checksum >> 8) & 0xFF; // High byte

  //Serial1.write(hexMessage, byteArraySize);
  right.write(hexMessage, byteArraySize);
  left.write(hexMessage, byteArraySize);
  top.write(hexMessage, byteArraySize);
  
}

// Ensures entire display is useable
void setDisplayWindowBig(){

  // cc = 01 01 00 00 00 00 00 40 00 20
  String input = "A5 68 32 01 7B 01 0A 00 00 00 01 01 00 00 00 00 00 C0 00 20 00 00 AE";

  // Count number of values
  int byteArraySize = countHexValues(input);

  // Create byte array with size of input
  byte hexMessage[byteArraySize];
  
  // Convert input into hexMessage (byteArray)
  convertToByteArray(input, hexMessage, byteArraySize); 

  //20/21 Checksum
  // Calculate checksum
  int length = byteArraySize - 3; // Length excluding end bytes
  uint16_t checksum = calculateChecksum(hexMessage, length);
  hexMessage[byteArraySize - 3] = checksum & 0xFF;        // Low byte
  hexMessage[byteArraySize - 2] = (checksum >> 8) & 0xFF; // High byte

  front.write(hexMessage, byteArraySize);
  // Serial2.write(hexMessage, byteArraySize);
  // Serial3.write(hexMessage, byteArraySize);
  // top.write(hexMessage, byteArraySize);
  
}



// For printing text on the LED
int printText(String message, int rgb[], bool isLargeDisplay, byte* hexMessage) {

  int messageLength = message.length();
  byte messageinHex[messageLength];
  stringToByteArray(message, messageinHex, messageLength);

  // cc = 04 00 01 01 00 00 00 00 00 40 00 20 11 00 00 00 + text + 00
  String input = "A5 68 32 01 7B 01 00 00 00 00 04 00 01 01 00 00 00 00 00 40 00 20 11 00 00 00 00 00 AE";
  // A5 68 32 01 7B 01 00 00 00 00 04 00 01 01 00 00 00 00 00 40 00 20 11 00 00 00 + text + 00 00 00 AE
  // 00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 + text + 
  // xx xx xx xx xx xx LL LL xx xx CC WW Ty Al xx xx yy yy wH wL hH hL Fo rr gg bb + text + 00 + csL csH End 

  int byteArraySize = countHexValues(input);
  byte initHexMessage[byteArraySize];
  convertToByteArray(input, initHexMessage, byteArraySize); // Converts input into hexMessage (byteArray)

  int length_array_ = sizeof(initHexMessage) / sizeof(initHexMessage[0]); // Length of the entire hexMessage
  
  int startBytesLength = 26;
  int endBytesLength = 4;

  byte startBytes[startBytesLength]; // For bytes upto the text
  byte endBytes[endBytesLength];     // For bytes after the text

  int hexMessageLength = startBytesLength + messageLength + endBytesLength; // Length of the entire hexMessage (Should be)

  // Copying required bytes from original message
  memcpy(startBytes, initHexMessage, startBytesLength);
  memcpy(endBytes, &initHexMessage[length_array_ - endBytesLength], endBytesLength);

  // Combining all to get new Hex Message
  memcpy(hexMessage, startBytes, startBytesLength);
  memcpy(&hexMessage[startBytesLength], messageinHex, messageLength);
  memcpy(&hexMessage[startBytesLength + messageLength], endBytes, endBytesLength);

  int length = hexMessageLength - 3;   // Length of hexMessage minus the ending code and checksum

  // Alignment (0:Left Aligned, 1:Center Aligned, 2:Right Aligned)
  hexMessage[13] = 01;

  if (isLargeDisplay) {

    // Image dimensions
    hexMessage[18] = 0x00; // Width (High byte)
    hexMessage[19] = 0xC0; // Width (Low byte) // 128 = 80, 64 = 40, 192 = C0
    hexMessage[20] = 0x00; // Height (High byte)
    hexMessage[21] = 0x20; // Height (Low byte) //64 = 40

    if (messageLength > 12){
        
      // X and Y Positions 
      hexMessage[14] = 0x00; // X position (High byte)
      hexMessage[15] = 0x00; // X position (Low byte)
      hexMessage[16] = 0x00; // Y position (High byte)
      hexMessage[17] = 0x05; // Y position (Low byte)
  
      // Font 
      hexMessage[22] = 0x03;
      
    } else {
  
      // X and Y Positions 
      hexMessage[14] = 0x00; // X position (High byte)
      hexMessage[15] = 0x00; // X position (Low byte)
      hexMessage[16] = 0x00; // Y position (High byte)
      hexMessage[17] = 0x00; // Y position (Low byte)
    
      // Font 
      hexMessage[22] = 0x04; // 12 Chars (0x04)
    
    }
    
  } else {

    // Image dimensions
    hexMessage[18] = 0x00; // Width (High byte)
    hexMessage[19] = 0x40; // Width (Low byte) // 64 (Length)
    hexMessage[20] = 0x00; // Height (High byte)
    hexMessage[21] = 0x20; // Height (Low byte) // 32 (Height)

    /// Small Screen Logic
    if (messageLength > 5 && messageLength < 9){
  
      // X and Y Positions 
      hexMessage[14] = 0x00; // X position (High byte)
      hexMessage[15] = 0x00; // X position (Low byte)
      hexMessage[16] = 0x00; // Y position (High byte)
      hexMessage[17] = 0x07; // Y position (Low byte)
  
      // Font 
      hexMessage[22] = 0x02; 
      
    } else if (messageLength == 5) {
  
      // X and Y Positions 
      hexMessage[14] = 0x00; // X position (High byte)
      hexMessage[15] = 0x00; // X position (Low byte)
      hexMessage[16] = 0x00; // Y position (High byte)
      hexMessage[17] = 0x00; // Y position (Low byte)
  
      // Font 
      hexMessage[22] = 0x03;
      
    } else if (messageLength < 5) {
  
      // X and Y Positions 
      hexMessage[14] = 0x00; // X position (High byte)
      hexMessage[15] = 0x00; // X position (Low byte)
      hexMessage[16] = 0x00; // Y position (High byte)
      hexMessage[17] = 0x00; // Y position (Low byte)
  
      // Font 
      hexMessage[22] = 0x04;
      
    } else {

      // Review else actions
      // X and Y Positions 
      hexMessage[14] = 0x00; // X position (High byte)
      hexMessage[15] = 0x00; // X position (Low byte)
      hexMessage[16] = 0x00; // Y position (High byte)
      hexMessage[17] = 0x0A; // Y position (Low byte)
  
      // Font 
      hexMessage[22] = 0x01;
      
    }

    if (message.indexOf("\n") != -1){
  
    // X and Y Positions 
    hexMessage[14] = 0x00; // X position (High byte)
    hexMessage[15] = 0x00; // X position (Low byte)
    hexMessage[16] = 0x00; // Y position (High byte)
    hexMessage[17] = 0x00; // Y position (Low byte)

    // Font 
    hexMessage[22] = 0x02;
    
    }
    
  }

  if (message == "GIVE WAY"){

    // Color
    hexMessage[23] = (byte)255;  // R
    hexMessage[24] = (byte)255;  // G
    hexMessage[25] = (byte)255;  // B
    
  } else {

    // Color
    hexMessage[23] = (byte)rgb[0];  // R
    hexMessage[24] = (byte)rgb[1];  // G
    hexMessage[25] = (byte)rgb[2];  // B
    
  }

  // Packet Length
  int packet_length = hexMessageLength - 13;   // CC length (13 is everything before CC + checksum (2) + end code
  byte lowByte = packet_length & 0xFF;         // Get the low byte (least significant byte)
  byte highByte = (packet_length >> 8) & 0xFF; // Get the high byte (most significant byte)
  hexMessage[6] = lowByte;
  hexMessage[7] = highByte;

  // Checksum
  uint16_t checksum_lower;
  uint16_t checksum_higher;
  uint16_t checksum = calculateChecksum(hexMessage, length);
  checksum_lower = checksum % 256;  
  checksum_higher = checksum / 256;

  hexMessage[hexMessageLength-3] = checksum_lower;
  hexMessage[hexMessageLength-2] = checksum_higher;

//  Serial.println("New HexMessage : ");
//  printByteArray(hexMessage, sizeof(hexMessage));
//  delay(10);

  return hexMessageLength;
  
}


// For printing Arrows on small panels
int printArrow (bool isRightArrow, int duration, byte* hexMessage) {

  String input = "A5 68 32 01 7B 01 1B 00 00 00 03 00 00 00 00 00 AE";
  int byteArraySize = countHexValues(input);
  byte initHexMessage[byteArraySize];
  convertToByteArray(input, initHexMessage, byteArraySize); // Converts input into hexMessage (byteArray)

  int startBytesLength = 29;
  int endBytesLength = 3;

  byte startBytes[startBytesLength]; // For bytes up to the image data
  byte endBytes[endBytesLength];    // For bytes after the image data

  int arrowDataLength;

  if (isRightArrow) {

    arrowDataLength = sizeof(rightArrow);
    
  } else {

    arrowDataLength = sizeof(leftArrow);
    
  }

  // Combining everything to create a new hex message
  int hexMessageLength = startBytesLength + arrowDataLength + endBytesLength;
  // byte hexMessage[hexMessageLength];

  // Copying required bytes from original message
  memcpy(startBytes, initHexMessage, startBytesLength);
  memcpy(endBytes, &initHexMessage[byteArraySize - endBytesLength], endBytesLength);

  if (isRightArrow) {

    // Combining all to get the new Hex Message
    memcpy(hexMessage, startBytes, startBytesLength);
    memcpy(&hexMessage[startBytesLength], rightArrow, arrowDataLength);
    memcpy(&hexMessage[startBytesLength + arrowDataLength], endBytes, endBytesLength);
    
  } else {

    // Combining all to get the new Hex Message
    memcpy(hexMessage, startBytes, startBytesLength);
    memcpy(&hexMessage[startBytesLength], leftArrow, arrowDataLength);
    memcpy(&hexMessage[startBytesLength + arrowDataLength], endBytes, endBytesLength);
        
  }

  // Set window and metadata for the graphical data
  hexMessage[11] = 0x00; // Window number (valid values: 0x00 to 0x07)
  hexMessage[12] = 0x00; // Mode: Draw
  hexMessage[13] = 0x00; // Speed (1 is the fastest; adjust as needed)

  // Display duration not consistent with the provided value
  // int duration = data.duration;
  byte time_lowByte = duration & 0xFF;         // Get the low byte (least significant byte)
  byte time_highByte = (duration >> 8) & 0xFF; // Get the high byte (most significant byte)
  hexMessage[14] = time_highByte; // Stay time (High byte)
  hexMessage[15] = time_lowByte; // Stay time (Low byte) -> 10 seconds
  
  hexMessage[16] = 0x04; // Format: Simple picture data

  // X and Y positions
  hexMessage[17] = 0x00; // X position (Low byte)
  hexMessage[18] = 0x00; // X position (High byte)
  hexMessage[19] = 0x00; // Y position (Low byte)
  hexMessage[20] = 0x00; // Y position (High byte)

  // Identify field
  hexMessage[21] = 0x49; // ASCII 'I'
  hexMessage[22] = 0x31; // ASCII '1'

  // Image dimensions
  hexMessage[23] = 0x40; // Width (Low byte)
  hexMessage[24] = 0x00; // Width (High byte)
  hexMessage[25] = 0x20; // Height (Low byte)
  hexMessage[26] = 0x00; // Height (High byte)

  // Property
  hexMessage[27] = 0x02; // 0x01 - Red, 0x02 - Green, 0x04 - Blue

  // Reserved byte
  hexMessage[28] = 0x00; // Reserved byte, always 0x00

  // Packet Length (6/7)
  int packet_length = startBytesLength + arrowDataLength - 12; // Exclude start and checksum bytes
  hexMessage[6] = packet_length & 0xFF;        // Low byte
  hexMessage[7] = (packet_length >> 8) & 0xFF; // High byte

  // Calculate checksum
  int length = hexMessageLength - 3; // Length excluding end bytes
  uint16_t checksum = calculateChecksum(hexMessage, length);
  hexMessage[hexMessageLength - 3] = checksum & 0xFF;        // Low byte
  hexMessage[hexMessageLength - 2] = (checksum >> 8) & 0xFF; // High byte

//  Serial.println("New HexMessage : ");
//  printByteArray(hexMessage, sizeof(hexMessage));
//  delay(10);

  return hexMessageLength;
  
}



// For printing solid colour on top panel
int printSolidColor(int duration, int rgb[], byte* hexMessage) {

  // Solid color on rooftop panel
  byte fullScreenData[258]; // (64x32=256) 64x32 panel
  for (int i = 0; i < 258; i++) {
      fullScreenData[i] = 0xFF; // All bits ON
  }

  String input = "A5 68 32 01 7B 01 1B 00 00 00 03 00 00 00 00 00 AE";
  int byteArraySize = countHexValues(input);
  byte initHexMessage[byteArraySize];
  convertToByteArray(input, initHexMessage, byteArraySize); // Converts input into hexMessage (byteArray)

  int startBytesLength = 29;
  int endBytesLength = 3;

  byte startBytes[startBytesLength]; // For bytes up to the image data
  byte endBytes[endBytesLength];    // For bytes after the image data

  int fullScreenDataLength = sizeof(fullScreenData);

  // Combining everything to create a new hex message
  int hexMessageLength = startBytesLength + fullScreenDataLength + endBytesLength;
  //byte hexMessage[hexMessageLength];

  // Copying required bytes from original message
  memcpy(startBytes, initHexMessage, startBytesLength);
  memcpy(endBytes, &initHexMessage[byteArraySize - endBytesLength], endBytesLength);

  // Combining all to get the new Hex Message
  memcpy(hexMessage, startBytes, startBytesLength);
  memcpy(&hexMessage[startBytesLength], fullScreenData, fullScreenDataLength);
  memcpy(&hexMessage[startBytesLength + fullScreenDataLength], endBytes, endBytesLength);

  // Set window and metadata for the graphical data
  hexMessage[11] = 0x00; // Window number (valid values: 0x00 to 0x07)
  hexMessage[12] = 0x00; // Mode: Draw
  hexMessage[13] = 0x00; // Speed (1 is the fastest; adjust as needed)

  // Display duration not consistent with the provided value
  // int duration = data.duration;
  byte time_lowByte = duration & 0xFF;         // Get the low byte (least significant byte)
  byte time_highByte = (duration >> 8) & 0xFF; // Get the high byte (most significant byte)
  hexMessage[14] = time_highByte; // Stay time (High byte)
  hexMessage[15] = time_lowByte; // Stay time (Low byte) -> 10 seconds
  
  hexMessage[16] = 0x04; // Format: Simple picture data

  // X and Y positions
  hexMessage[17] = 0x00; // X position (Low byte)
  hexMessage[18] = 0x00; // X position (High byte)
  hexMessage[19] = 0x00; // Y position (Low byte)
  hexMessage[20] = 0x00; // Y position (High byte)

  // Identify field
  hexMessage[21] = 0x49; // ASCII 'I'
  hexMessage[22] = 0x31; // ASCII '1'

  // Image dimensions
  hexMessage[23] = 0x40; // Width (Low byte)
  hexMessage[24] = 0x00; // Width (High byte)
  hexMessage[25] = 0x20; // Height (Low byte)
  hexMessage[26] = 0x00; // Height (High byte)

  // Property - color
  if (rgb[0] == 255 && rgb[1] == 0 && rgb[2] == 0){
    // Red
    hexMessage[27] = 0x01;
  } else if (rgb[0] == 0 && rgb[1] == 128 && rgb[2] == 0){
    // Green
    hexMessage[27] = 0x02;
  } else if (rgb[0] == 0 && rgb[1] == 0 && rgb[2] == 255){
    // Blue
    hexMessage[27] = 0x04;
  } else {
    // ?
    hexMessage[27] = 0x02;
  }

  // Property
  // hexMessage[27] = 0x01; // 0x01 - Red, 0x02 - Green, 0x04 - Blue

  // Reserved byte
  hexMessage[28] = 0x00; // Reserved byte, always 0x00

  // Packet Length (6/7)
  int packet_length = startBytesLength + fullScreenDataLength - 12; // Exclude start and checksum bytes
  hexMessage[6] = packet_length & 0xFF;        // Low byte
  hexMessage[7] = (packet_length >> 8) & 0xFF; // High byte

  // Calculate checksum
  int length = hexMessageLength - 3; // Length excluding end bytes
  uint16_t checksum = calculateChecksum(hexMessage, length);
  hexMessage[hexMessageLength - 3] = checksum & 0xFF;        // Low byte
  hexMessage[hexMessageLength - 2] = (checksum >> 8) & 0xFF; // High byte

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
  
