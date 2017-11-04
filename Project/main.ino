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
#define ENC1            0x02                                  // Motor encoder 1 byte
#define ENC2            0x06                                  // Motor encoder 2 byte
#define ACCEL           0x0E                                  // Motor acceleration byte
#define CMD             0x10                                  // Command byte (Used for reset)
#define MODE            0x0F                                  // Control mode switch byte

int speed = 0;                                                // Combined speed

const int mode = 2;                                           // Motor mode 2 - [Reverse] 0 <-> 128 <-> 255 [Forward]
const int ledPin = 7;                                         // LED Signal pin
const int servoPin = 9;                                       // Drop servo pin

float d1 = 0;
float d2 = 1;

Servo dropServo;                                              // Define servo
int cpos = 0;                                                 // Declare initial (current) servo position

/*
------------------------------------------------------------------------------------------------------------------------
 Define setup function. Arduino will run this code once when first turned on.
------------------------------------------------------------------------------------------------------------------------
*/

void setup(){
    pinMode(ledPin, OUTPUT);                                  // Sets LED pin mode to OUTPUT
    dropServo.attach(servoPin);                               // Attach servo to servo pin
    resetServo();                                             // Reset servo to 0 position

    Wire.begin();                                             // Begin I2C bus
    Serial.begin();                                           // Begin serial monitoring for debug

    Wire.beginTransmission(MD25);                             // Begin communication with MD25
    Wire.write(MODE);                                         // Write to mode switch
    Wire.write(mode);                                         // Write mode
    Wire.endTransmission();                                   // Send command, stop communication

    encodeValuesReset();                                      // Reset encoder values

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
    int pos = cpos;                                           // Declare the current position
    for (pos = cpos; pos <= npos; pos +=1){
        myservo.write(pos);
        delay(15);
    }
    cpos = pos;                                               // Set current position to servo position
}

// Function to reset servo to zero position
void resetServo(){
    cpos = servo.read();                                      // Get current servo position
    for (pos = cpos; pos > 0; pos -=1){
        myservo.write(pos);
        delay(15);
    }
    cpos = pos;
}


// Function to read and display value of encoder 1 as a float
float encoder1(){
    Wire.beginTransmission(MD25);                             // MD25 address
    Wire.write(ENC1);                                         // Read the position of encoder 1
    Wire.endTransmission();

    Wire.requestFrom(MD25, 4); 	                              // Request 4 bytes from MD25
    while(Wire.available() < 4);                              // Wait for 4 bytes to arrive
    long poss1 = Wire.read();                                 // First byte for encoder 1, HH.
    poss1 <<= 8;
    poss1 += Wire.read();                                     // Second byte for encoder 1, HL
    poss1 <<= 8;
    poss1 += Wire.read();                                     // Third byte for encoder 1, LH
    poss1 <<= 8;
    poss1 +=Wire.read();                                      // Final byte
    poss1_mm = poss1*0.093;                                   // Convert to mm
    return(poss1_mm);
}

// Function to read and display value of encoder 2 as a float
float encoder2(){
    Wire.beginTransmission(MD25);                             // MD25 address
    Wire.write(ENC2);                                         // Read the position of encoder 1
    Wire.endTransmission();

    Wire.requestFrom(MD25, 4); 	                              // Request 4 bytes from MD25
    while(Wire.available() < 4);                              // Wait for 4 bytes to arrive
    long poss1 = Wire.read();                                 // First byte for encoder 1, HH.
    poss1 <<= 8;
    poss1 += Wire.read();                                     // Second byte for encoder 1, HL
    poss1 <<= 8;
    poss1 += Wire.read();                                     // Third byte for encoder 1, LH
    poss1 <<= 8;
    poss1 +=Wire.read();                                      // Final byte
    poss1_mm = poss1*0.093;                                   // Convert to mm
    return(poss1_mm);
}

// Function to light an LED when a step has been completed
void ledNotify(){
    digitalWrite(ledPin, HIGH);
    delay(20);
    digitalWrite(ledPin, LOW);
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

// Function to stop motors and check for overshoot
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
/*
------------------------------------------------------------------------------------------------------------------------
 Arduino will run this after setup.
------------------------------------------------------------------------------------------------------------------------
*/
void loop(){
    moveForward(100);
    dispense();
    ledNotify();
}
