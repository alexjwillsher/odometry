/*
   ____      _                      _                _______        _      ___   ___  __ ______
  / __ \    | |                    | |              |__   __|      | |    |__ \ / _ \/_ |____  |
 | |  | | __| | ___  _ __ ___   ___| |_ _ __ _   _     | | __ _ ___| | __    ) | | | || |   / /
 | |  | |/ _` |/ _ \| '_ ` _ \ / _ \ __| '__| | | |    | |/ _` / __| |/ /   / /| | | || |  / /
 | |__| | (_| | (_) | | | | | |  __/ |_| |  | |_| |    | | (_| \__ \   <   / /_| |_| || | / /
  \____/ \__,_|\___/|_| |_| |_|\___|\__|_|   \__, |    |_|\__,_|___/_|\_\ |____|\___/ |_|/_/
                                              __/ |
                                             |___/

Group P: Arthur Thirion, Alexander Timms, Alexander Willsher, Andrew Mulligan, Evan Heui

------------------------------------------------------------------------------------------------------------------------
 Includes, definitions and declarations.
------------------------------------------------------------------------------------------------------------------------
*/
#include <Wire.h>                                             // Include I2C bus library
#include <Servo.h>                                            // Include Servo Library

#define MD25            0x58                                  // MD25 Address
#define SPEED           0x00                                  // Motor speed byte
#define TURN            0x01                                  // Motor Turn byte (speed2)
#define ENC1            0x02                                  // Motor encoder 1 byte
#define ENC2            0x06                                  // Motor encoder 2 byte
#define ACCEL           0x0E                                  // Motor acceleration byte
#define CMD             0x10                                  // Command byte (Used for reset)
#define MODE            0x0F                                  // Control mode switch byte

const int ledPin = 12;                                        // LED Signal pin
const int piezoPin = 10;                                      // Piezo pin
const int servoPin = 9;                                       // Drop servo pin

const float robotWidth = 27.5;
const float pi = 3.14159265359;

float d1 = 0;
float d2 = 1;

Servo dropServo;                                              // Define servo
int cpos = 0;                                                 // Declare initial (current) servo position
int pos = 0;                                                  // Initialize servo position variable

/*
------------------------------------------------------------------------------------------------------------------------
 Define setup function. Arduino will run this code once when first turned on.
------------------------------------------------------------------------------------------------------------------------
*/

void setup(){
    pinMode(ledPin, OUTPUT);                                  // Sets LED pin mode to OUTPUT
    digitalWrite(ledPin, LOW);
    noTone(piezoPin);
    dropServo.attach(servoPin);                               // Attach servo to servo pin
    resetServo();                                             // Reset servo to 0 position

    Serial.begin(9600);

    Wire.begin();                                             // Begin I2C bus
    setMode(2);
    setAccel(4);
    encoderValuesReset();                                     // Reset encoder values
}


/*
------------------------------------------------------------------------------------------------------------------------
 Define functions to be called.
------------------------------------------------------------------------------------------------------------------------
*/
// Function to reset encoder values
void encoderValuesReset(){
    Wire.beginTransmission(MD25);                             // Begin communication with MD25
    Wire.write(CMD);                                          // Write to Command
    Wire.write(0x20);                                         // Reset encoder registers to zero
    Wire.endTransmission();                                   // Send command, stop communication
    delay(30);
}

// Function to turn servo by 30 degrees and dispense M&M
void dispense(){
    int npos = cpos + 30;                                     // Declare the target position
    for (pos = cpos; pos <= npos; pos +=1){
        dropServo.write(pos);
        delay(30);
    }
    cpos = pos;                                               // Set current position to servo position
}

void penultimateDispense(){
    int npos = cpos + 25;                                     // Declare the target position
    for (pos = cpos; pos <= npos; pos +=1){
        dropServo.write(pos);
        delay(30);
    }
    cpos = pos;                                               // Set current position to servo position
}

// Function to reset servo to zero position
void resetServo(){
    dropServo.write(0);
    //cpos = dropServo.read();                                  // Get current servo position
    //for (pos = cpos; pos >= 0; pos -=1){
    //    dropServo.write(pos);
    //    delay(15);
    //}
    //cpos = pos;
}


// Function to read and display value of encoder 1 as a float
float encoder1(){
    Wire.beginTransmission(MD25);                             // MD25 address
    Wire.write(ENC1);                                         // Read the position of encoder 1
    Wire.endTransmission();

    Wire.requestFrom(MD25, 4);                                // Request 4 bytes from MD25
    while(Wire.available() < 4);                              // Wait for 4 bytes to arrive
    long poss1 = Wire.read();                                 // First byte for encoder 1, HH.
    poss1 <<= 8;
    poss1 += Wire.read();                                     // Second byte for encoder 1, HL
    poss1 <<= 8;
    poss1 += Wire.read();                                     // Third byte for encoder 1, LH
    poss1 <<= 8;
    poss1 +=Wire.read();                                      // Final byte
    return(poss1*0.09);                                     // Convert to cm
}

// Function to read and display value of encoder 2 as a float
float encoder2(){
    Wire.beginTransmission(MD25);                             // MD25 address
    Wire.write(ENC2);                                         // Read the position of encoder 1
    Wire.endTransmission();

    Wire.requestFrom(MD25, 4);                                // Request 4 bytes from MD25
    while(Wire.available() < 4);                              // Wait for 4 bytes to arrive
    long poss1 = Wire.read();                                 // First byte for encoder 1, HH.
    poss1 <<= 8;
    poss1 += Wire.read();                                     // Second byte for encoder 1, HL
    poss1 <<= 8;
    poss1 += Wire.read();                                     // Third byte for encoder 1, LH
    poss1 <<= 8;
    poss1 +=Wire.read();                                      // Final byte
    return(poss1*0.09);                                      // Convert to cm
}

// Function to light an LED and sound a tone when a step has been completed
void checkpointNotify(){
    analogWrite(ledPin, 255);
    tone(piezoPin, 2000);
    delay(100);
    tone(piezoPin, 3000);
    delay(100);
    tone(piezoPin, 4000);
    delay(100);
    tone(piezoPin, 5000);
    delay(100);
    noTone(10);
    digitalWrite(ledPin, LOW);
    delay(100);
}

// Function to move the robot forward by the amount x at speed speed
void moveForward(float x, int speed){
    do {
        Wire.beginTransmission(MD25);
        Wire.write(SPEED);
        Wire.write(speed);
        Wire.endTransmission();
    }while(encoder1() < x-3);
    stopMotors();
}

// Function to turn robot on the spot
void rotateRight(float degrees, int v){
    float wDist = (degrees/360)*robotWidth*pi;
    do{
        Wire.beginTransmission(MD25);
        Wire.write(TURN);
        Wire.write(v);
        Wire.endTransmission();
    }while(encoder1() < wDist-3.5);    
    stopMotors();   
}

// Function to turn robot on the spot
void rotateLeft(float degrees, int v){
    float wDist = (degrees/360)*robotWidth*pi;
    do{
        Wire.beginTransmission(MD25);
        Wire.write(TURN);
        Wire.write(-v);
        Wire.endTransmission();
    }while(encoder2() < wDist-3.5);    
    stopMotors();   
}

// Function to move robot in arc
void arc(float w1d, float w2d, int v){
    float ratio = w1d/w2d;
    int w1v = (int) v*ratio;
    setMode(1);
    do{
        Wire.beginTransmission(MD25);
        Wire.write(SPEED);
        Wire.write(w1v);
        Wire.endTransmission();

        Wire.beginTransmission(MD25);
        Wire.write(TURN);
        Wire.write(v);
        Wire.endTransmission();
    }while(encoder2()<w2d-5.5);
    setMode(2);
    stopMotors();
}


// Function to stop motors
void stopMotors(){
  setAccel(6);
    do{
        d1 = encoder2();
        delay(5);
        d2 = encoder2();
        Wire.beginTransmission(MD25);
        Wire.write(SPEED);
        Wire.write(128);
        Wire.endTransmission();
        Wire.beginTransmission(MD25);
        Wire.write(TURN);
        Wire.write(128);
        Wire.endTransmission();
    }while(d1 != d2);
    encoderValuesReset();
    setAccel(2);
}

// Function to change motor mode
void setMode(int mode){
    Wire.beginTransmission(MD25);                             // Begin communication with MD25
    Wire.write(MODE);                                         // Write to mode switch
    Wire.write(mode);                                         // Write mode
    Wire.endTransmission();                                   // Send command, stop communication
}

void setAccel(int accel){
    Wire.beginTransmission(MD25);                             // Begin communication with MD25
    Wire.write(ACCEL);                                        // Write to mode switch
    Wire.write(accel);                                        // Write mode
    Wire.endTransmission();                                   // Send command, stop communication
}
/*
------------------------------------------------------------------------------------------------------------------------
 Arduino will run this after setup.
------------------------------------------------------------------------------------------------------------------------
*/
void loop(){
  delay(2000);
//13 to 12
    moveForward(41, 160);
    checkpointNotify();
    
//12 to 11
    moveForward(38, 160);
    checkpointNotify();

   
    
//11 turn to 10
    rotateRight(138, 160);
    arc(26.4, 155, 80);
    dispense();
    checkpointNotify();
    

    
//10 turn to 9
    rotateLeft(85, 180);
    moveForward(13, 200);
    checkpointNotify();
    
//9 turn to 8
    rotateRight(144, 180);
    moveForward(56, 200);
    dispense();
    checkpointNotify();
    
    
//8 turn to 7
    rotateRight(43, 180);
    moveForward(34, 200);
    checkpointNotify();
    
//7 turn to 6
    rotateRight(88, 180);
    moveForward(35, 200);
    dispense();
    checkpointNotify();
    
//6 turn to 5
    rotateRight(90, 180);
    moveForward(34, 200);
    checkpointNotify();
  
//5 turn to 4
    rotateRight(85, 180);
    moveForward(60, 200);
    penultimateDispense();
    checkpointNotify();

//4 arc turn to 3
    rotateLeft(90, 180);
    arc(17.8, 60.8, 80);
    checkpointNotify();
    
//3 turn to 2
    rotateRight(88, 180);
    moveForward(43, 180);
    dispense();
    checkpointNotify();

    
//2 turn to 1
    rotateRight(90, 180);
    moveForward(20, 200);
    checkpointNotify();
//1 turn to 13
    rotateLeft(90, 180);
    moveForward(29, 200);
    checkpointNotify();
    

    resetServo();
    delay(99999);
}
