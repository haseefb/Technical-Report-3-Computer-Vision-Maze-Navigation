#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Keypad.h>
#include <math.h>  //include math library for use with MPU6050
#include <MPU6050_tockn.h>  //include MPU6050 library
#define I2C_SLAVE_ADDR  0x04 //Define the I2C address of the Arduino Nano


// Define OLED display pins
#define OLED_SDA 21
#define OLED_SCL 22
#define OLED_ADDR 0x3C // OLED display address
// Define the OLED display object
Adafruit_SSD1306 display(128, 64, &Wire, OLED_SDA, OLED_SCL);
MPU6050 mpu6050(Wire);

int leftMotor_speed, rightMotor_speed, servoAngle;  //variables to store the motor speeds and servo angle
int x = 0;  //variable to store data for transmission

// Define the keypad pins and keys
const byte ROWS = 4;
const byte COLS = 3;
char keys[ROWS][COLS] = {
  {'3', '6', '9'},
  {'2', '5', '8'},
  {'1', '4', '7'},
  {'*', '0', '#'}
};
byte rowPins[ROWS] = {2, 0, 4, 16};
byte colPins[COLS] = {18, 5, 17};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// Define variables for storing the key sequence
char keySequence[50];
int sequenceLength = 0;
 

//Set constants for pi, diameter of wheel (D) and number of encoders counts of revolution (N)
const float pi = 3.14;
const float D = 5.9;
const float N = 25;
int angle = 0;
float angle_change = 0;

//Sequence Loop count
int loopCount = 0; // Initialize the loop count to 1
int loopCounts[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void setup() {
  
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize OLED display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // Initialize the OLED display
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  // Display initial message
  display.setCursor(0, 0);
  display.println("   Enter sequence  ");
  display.println("2 = F     | 8 = B");
  display.println("4 = L     | 6 = R");
  display.println("7 = clear | 9 = S");
  display.println("5 = execute");
  display.display();  // Update the display with the new content  
  // Initialize I2C communication
  Wire.begin(); 
  mpu6050.begin();// This line initializes the MPU-6050 using the begin function from the MPU6050 library
  mpu6050.calcGyroOffsets(true);// This line uses the calcGyroOffsets function to calibrate the gyroscope in the MPU-6050. 
  
}

void loop() {

  // Read the keypad input
  char key = keypad.getKey();
  display.setTextColor(WHITE);
  // Debug
  if (key != NO_KEY) {
    // Display the key on the OLED display
    display.clearDisplay();
    display.setCursor(0, 10);
    display.print("Key pressed: ");
    display.println(key);
    display.display();

    if (key) {
      Serial.println(key);
    }
  }

  if (key == '2') {
    keySequence[sequenceLength] = 'F';
     int loopCountF = 0;
          while (loopCountF == 0) {  // Loop until a valid input is received
            display.clearDisplay();
            display.setCursor(0, 0);
            display.println("Enter Distance");
            display.println("1 = 10cm | 9 = 90cm");            
            display.display();
            char loopCountCharF = keypad.getKey();
            if (loopCountCharF >= '1' && loopCountCharF <= '9') {  // Check if the input is a number
              loopCountF = loopCountF * 10 + (loopCountCharF - '0');  // Update the loop count
              display.clearDisplay();
              display.setCursor(0, 0);
              display.print("Distance: ");
              display.print(loopCountF);
              display.print("0 cm");
              display.display();
            }
          }
    loopCounts[sequenceLength] = loopCountF;
    sequenceLength++;
  } else if (key == '4') {
    keySequence[sequenceLength] = 'L';
    int loopCountL = 0;
        while (loopCountL == 0) {  // Loop until a valid input is received
            display.clearDisplay();
            display.setCursor(0, 0);
            display.println("Enter Rotation");
            display.println("1 = 90deg 2 = 180deg ");            
            display.display();
            char loopCountCharL = keypad.getKey();
            if (loopCountCharL >= '1' && loopCountCharL <= '2') {  // Check if the input is a number
              loopCountL = loopCountL * 10 + (loopCountCharL - '0');  // Update the loop count
              display.clearDisplay();
              display.setCursor(0, 0);
              display.print("Rotate: ");
              display.print(loopCountL);
              display.print("0 deg");
              display.display();
            }
          }
    loopCounts[sequenceLength] = loopCountL;
    sequenceLength++;
  } else if (key == '6') {
    keySequence[sequenceLength] = 'R';
        int  loopCountR = 0;
            while (loopCountR == 0) {  // Loop until a valid input is received
            display.clearDisplay();
            display.setCursor(0, 0);
            display.println("Enter Rotation");
            display.println("1 = 90deg 2 = 180deg ");            
            display.display();
            char loopCountCharR = keypad.getKey();
            if (loopCountCharR >= '1' && loopCountCharR <= '2') {  // Check if the input is a number
              loopCountR = loopCountR * 10 + (loopCountCharR - '0');  // Update the loop count
              display.clearDisplay();
              display.setCursor(0, 0);
              display.print("Rotate: ");
              display.print(loopCountR);
              display.print("0 deg");
              display.display();
            }
          }
    loopCounts[sequenceLength] = loopCountR;      
    sequenceLength++;
  } else if (key == '8') {
    keySequence[sequenceLength] = 'B';
  int loopCountB = 0;    
         while (loopCountB == 0) {  // Loop until a valid input is received
            display.clearDisplay();
            display.setCursor(0, 0);
            display.println("Enter Distance");
            display.println("1 = 10cm | 9 = 90cm");         
            display.display();
            char loopCountCharB = keypad.getKey();
            if (loopCountCharB >= '1' && loopCountCharB <= '9') {  // Check if the input is a number
              loopCountB = loopCountB * 10 + (loopCountCharB - '0');  // Update the loop count
             display.clearDisplay();
              display.setCursor(0, 0);
              display.print("Distance: ");
              display.print(loopCountB);
              display.print("0 cm");
              display.display();
            }
          }
    loopCounts[sequenceLength] = loopCountB;      
    sequenceLength++;
  } else if (key == '9') {
    keySequence[sequenceLength] = 'S';
    sequenceLength++;
  } else if (key == '5') {
    // Execute the key sequence and display a message on the OLED display
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Executing command");
    display.display();
    
    for (int i = 0; i < sequenceLength; i++) {
      char command = keySequence[i];
      int loopCount = loopCounts[i]; // get the loop count for the current command
      switch (command) {
        case 'F':
          for (int i = 0; i < loopCount; i++) {  // Execute the loop
            goForward();  
            measure_distance("forward", 10);      
            stop(); 
          }
          break;   
        case 'L':
          for (int i = 0; i < loopCount; i++) {  // Execute the loop
          goLeft();
          turn("Left");
          delay(10);
          stop();
          }
          break;
          
        case 'R':
          for (int i = 0; i < loopCount; i++) {  // Execute the loop
          goRight();
          turn("Right");
          delay(10);
          stop();
          }
          break;
          
        case 'B':     
          for (int i = 0; i < loopCount; i++) {  // Execute the loop
            goBackward();
            measure_distance("backwards", 10);    
            stop();   
          }
          break;
          
        case 'S':
          stop();
          break;
      }
    }
    
    // Reset the key sequence
    sequenceLength = 0;
    display.clearDisplay();
    display.setCursor(0, 20);
    display.println("Command executed");
    
  } else if (key == '7') {
    // Clear the key sequence and display a message on the OLED display
    sequenceLength = 0;
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("sequence cleared");
          }
  }


      // Define motor control functions
      void goForward() {
        // Move the robot forward
        delay(1000);
        leftMotor_speed = 150;
        rightMotor_speed = 150;
        servoAngle = 90;
        Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
      }

      void goLeft() {
        // Code to turn the robot left
        delay(1000);
        leftMotor_speed = 100;
        rightMotor_speed = 250;
        servoAngle = -20;
        Serial.println("Going Left!");
       Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);  //transmit data to arduino
      }


      void goRight() {
        // Code to turn the robot right
        delay(1000);
        leftMotor_speed = 250;
        rightMotor_speed = 100;
        servoAngle = 130;
        Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);  //transmit data to arduino
        Serial.println("Going Right!"); 
    }

      void goBackward() {
        // Code to move the robot backward
        delay(1000);
        leftMotor_speed = -150;
        rightMotor_speed = -150;
        servoAngle = 88;
        Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);  //transmit data to arduino
        Serial.println("Going Backwards!");
      }

      void stop() {
        // Code to move the robot backward
        delay(1000);
        leftMotor_speed = 0;
        rightMotor_speed = 0;
        servoAngle = 88;
        Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);  //transmit data to arduino
        Serial.println("Stop!");
      }

 void measure_distance(char *direction, int distance)
  {
   
    long enc1_count, enc2_count;
    long enc1_count_new, enc2_count_new;
    int enc_change = 0;

    Wire.requestFrom(I2C_SLAVE_ADDR,2);
    if (Wire.available () >= 2)
    {
      enc1_count = Wire.read();
      enc2_count = Wire.read();
    }
   
    if (direction == "backward");
    {
       while (enc_change < distance)
      {
        Wire.requestFrom(I2C_SLAVE_ADDR,2);
        if (Wire.available () >= 2)
        {
          enc1_count_new = Wire.read();
          enc2_count_new = Wire.read();
          if (enc1_count >= enc1_count_new)
          {                            
            enc_change = (enc1_count - enc1_count_new);
            enc_change = enc_change * ((D*pi)/N);
          }
          else if (enc1_count < enc1_count_new)
          {                                  
            enc_change = ((enc1_count + 254) - enc1_count_new);
            enc_change = enc_change * ((D*pi)/N);
          }    
          if (enc_change < (distance - 10)) // done
          {
            enc_change = 0;
            enc1_count = enc1_count_new;
          }
          Serial.print("\enc1 : ");
          Serial.println(enc1_count_new);
          Serial.print("\enc1 og : ");
          Serial.println(enc1_count);
          Serial.print("\enc1 change : ");
          Serial.println(enc_change);
        }
      }
    }
    if (direction == "forward");
    {
      while (enc_change < distance)
      {
        Wire.requestFrom(I2C_SLAVE_ADDR,2);
        if (Wire.available () >= 2)
        {
          enc1_count_new = Wire.read();
          enc2_count_new = Wire.read();
          if (enc1_count >= enc1_count_new)
          {                            
            enc_change = (enc1_count - enc1_count_new);
            enc_change = enc_change * ((D*pi)/N);
          }
          else if (enc1_count < enc1_count_new)
          {                                  
            enc_change = ((enc1_count + 254) - enc1_count_new);
            enc_change = enc_change * ((D*pi)/N);
          }    
          if (enc_change < (distance - 10)) // done
          {
            enc_change = 0;
            enc1_count = enc1_count_new;
          }
          Serial.print("\enc1 : ");
          Serial.println(enc1_count_new);
          Serial.print("\enc1 og : ");
          Serial.println(enc1_count);
          Serial.print("\enc1 change : ");
          Serial.println(enc_change);
        }
      }
    }
  }
  void turn(char *turn_type){
    int angle = 0;
    float angle_change = 0;

    Wire.beginTransmission(I2C_SLAVE_ADDR);
    if (turn_type == "Left")
    {                                  
      goLeft();
    }
    else if (turn_type == "Right")
    {
      goRight();  
    }
    mpu6050.begin();
    angle = (mpu6050.getAngleZ());
    while (angle_change < 15)
    {
      mpu6050.update();
      Serial.print("\tangleZ : ");
      Serial.println(mpu6050.getAngleZ());
      if (turn_type == "Left")
      {
        angle_change = ((mpu6050.getAngleZ()) - angle);
      }
      else if (turn_type == "Right")
      {
        angle_change = (angle - (mpu6050.getAngleZ()));
      }
    }
  }

      
        void Transmit_to_arduino(int leftMotor_speed, int rightMotor_speed, int servoAngle){
        Wire.beginTransmission(I2C_SLAVE_ADDR); // transmit to device #4
        Wire.write((byte)((leftMotor_speed & 0x0000FF00) >> 8));    // first byte of leftMotor_speed, containing bits 16 to 9
        Wire.write((byte)(leftMotor_speed & 0x000000FF));           //leftMotor_speed, containing the 8 LSB - bits 8 to 1
        Wire.write((byte)((rightMotor_speed & 0x0000FF00) >> 8));   // first byte of rightMotor_speed, containing bits 16 to 9
        Wire.write((byte)(rightMotor_speed & 0x000000FF));          // second byte of rightMotor_speed, containing the 8 LSB - bits 8 to 1
        Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));         // first byte of servoAngle, containing bits 16 to 9
        Wire.write((byte)(servoAngle & 0x000000FF)); // second byte of servoAngle, containing the 8 LSB - bits 8 to 1
        Wire.endTransmission();   // stop transmitting
      }

