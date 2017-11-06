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

const int ledPin = 13;                                         // LED Signal pin
const int piezoPin = 10;                                      // Piezo pin
const int servoPin = 9;                                       // Drop servo pin

const float robotWidth = 28.0;
const float pi = 3.14159265359;

float d1 = 0;
float d2 = 1;

Servo dropServo;                                              // Define servo
int cpos = 0;                                                 // Declare initial (current) servo position
int pos = 0;                                                  // Initilize servo position variable

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

    Wire.begin();                                             // Begin I2C bus
    setMode(2);
    encoderValuesReset();                                      // Reset encoder values
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

//Function to turn servo by 30 degrees and dispense M&M
void dispense(){
    int npos = cpos + 30;                                     // Declare the target position
    for (pos = cpos; pos <= npos; pos +=1){
        dropServo.write(pos);
        delay(15);
    }
    cpos = pos;                                               // Set current position to servo position
}

// Function to reset servo to zero position
void resetServo(){
    cpos = dropServo.read();                                      // Get current servo position
    for (pos = cpos; pos > 0; pos -=1){
        dropServo.write(pos);
        delay(15);
    }
    cpos = pos;
}


// Function to read and display value of encoder 1 as a float
float encoder1(){
    Wire.beginTransmission(MD25);                             // MD25 address
    Wire.write(ENC1);                                         // Read the position of encoder 1
    Wire.endTransmission();

    Wire.requestFrom(MD25, 4);                                 // Request 4 bytes from MD25
    while(Wire.available() < 4);                              // Wait for 4 bytes to arrive
    long poss1 = Wire.read();                                 // First byte for encoder 1, HH.
    poss1 <<= 8;
    poss1 += Wire.read();                                     // Second byte for encoder 1, HL
    poss1 <<= 8;
    poss1 += Wire.read();                                     // Third byte for encoder 1, LH
    poss1 <<= 8;
    poss1 +=Wire.read();                                      // Final byte
    return(poss1*0.093);                                      // Convert to cm
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
    return(poss1*0.093);                                      // Convert to cm
}

// Function to light an LED and sound a tone when a step has been completed
void checkpointNotify(){
    analogWrite(ledPin, 255);
    tone(piezoPin, 3000);
    delay(100);
    tone(piezoPin, 4000);
    delay(100);
    noTone(10);
    digitalWrite(ledPin, LOW);
    delay(100);
    delay(5000);
}

// Function to move the robot forward by the amount x at speed speed
void moveForward(float x, int speed){
    do {
        Wire.beginTransmission(MD25);
        Wire.write(SPEED);
        Wire.write(speed);
        Wire.endTransmission();
    }while(encoder1() < x);
    stopMotors();
}

//Function to turn robot on the spot
void rotate(char direction, float degrees, int speed){
    double w1Dist = degrees/360*robotWidth*pi;
    if(direction == "ccw"){
        do{
            Wire.beginTransmission(MD25);
            Wire.write(TURN);
            Wire.write(speed);
            Wire.endTransmission();
        }while(encoder1()<w1Dist);
    }
    else if(direction == "cw"){
        do{
            Wire.beginTransmission(MD25);
            Wire.write(TURN);
            Wire.write(-speed);
            Wire.endTransmission();
        }while(encoder1()<w1Dist);
    }
    stopMotors();
}

// Function to stop motors
void stopMotors(){
    do{
        d1 = encoder1();
        delay(50);
        d2 = encoder1();
        Wire.beginTransmission(MD25);
        Wire.write(SPEED);
        Wire.write(128);
        Wire.endTransmission();
    }while(d1 != d2);

}

// Function to change motor mode
void setMode(int mode){
    Wire.beginTransmission(MD25);                             // Begin communication with MD25
    Wire.write(MODE);                                         // Write to mode switch
    Wire.write(mode);                                         // Write mode
    Wire.endTransmission();                                   // Send command, stop communication
}
/*
------------------------------------------------------------------------------------------------------------------------
 Arduino will run this after setup.
------------------------------------------------------------------------------------------------------------------------
*/
void loop(){
    moveForward(340, 200);
    checkpointNotify();

    rotate("cw", 90, 50);
    moveForward(260, 200);
    checkpointNotify();

    rotate("ccw", 90, 50);
    moveForward(500, 200);
    checkpointNotify();

    while(1);
}
