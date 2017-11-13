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
#include <Wire.h>                                       // Include I2C bus library
#include <Servo.h>                                      // Include Servo Library

#define MD25            0x58                            // MD25 Address
#define SPEED           0x00                            // Motor speed byte
#define TURN            0x01                            // Motor Turn byte (speed2)
#define ENC1            0x02                            // Motor encoder 1 byte
#define ENC2            0x06                            // Motor encoder 2 byte
#define ACCEL           0x0E                            // Motor acceleration byte
#define CMD             0x10                            // Command byte (Used for reset)
#define MODE            0x0F                            // Control mode switch byte

const int ledPin = 12;                                  // LED Signal pin
const int piezoPin = 10;                                // Piezo pin
const int servoPin = 9;                                 // Drop servo pin

const float robotWidth = 27.5;                          // Define robot width for arc
const float pi = 3.14159265359;                         // Define Pi for calculations
float d1 = 0;                                           // Initialise d1
float d2 = 1;                                           // Initialise d2

Servo dropServo;                                        // Define servo
int cpos = 0;                                           // Declare servo position
int pos = 0;                                            // Initialize servo position

/*
------------------------------------------------------------------------------------------------------------------------
 Define setup function. Arduino will run this code once when first turned on.
------------------------------------------------------------------------------------------------------------------------
*/

void setup(){
    pinMode(ledPin, OUTPUT);                            // Sets LED pin mode to OUTPUT
    digitalWrite(ledPin, LOW);
    noTone(piezoPin);
    dropServo.attach(servoPin);                         // Attach servo to servo pin
    resetServo();                                       // Reset servo to 0 position

    Serial.begin(9600);

    Wire.begin();                                       // Begin I2C bus
    setMode(2);                                         // Set mode 2 0 > 128 > 255
    setAccel(4);                                        // Set acceleration 4 (/10)
    encoderValuesReset();                               // Reset encoder values
}


/*
------------------------------------------------------------------------------------------------------------------------
 Define functions to be called.
------------------------------------------------------------------------------------------------------------------------
*/
// Function to reset encoder values
void encoderValuesReset(){
    Wire.beginTransmission(MD25);                       // Begin communication with MD25
    Wire.write(CMD);                                    // Write to Command
    Wire.write(0x20);                                   // Reset encoder registers to zero
    Wire.endTransmission();                             // Send command, stop communication
    delay(30);
}

// Function to turn servo by 30 degrees and dispense M&M
void dispense(){
    int npos = cpos + 30;                               // Declare the target position
    for (pos = cpos; pos <= npos; pos +=1){
        dropServo.write(pos);
        delay(30);
    }
    cpos = pos;                                         // Set position to servo position
}

// Function to turn servo by 25 degrees and dispense penultimate M&M
void penultimateDispense(){
    int npos = cpos + 25;                               // Declare the target position
    for (pos = cpos; pos <= npos; pos +=1){
        dropServo.write(pos);
        delay(30);
    }
    cpos = pos;                                         // Set position to servo position
}

// Function to reset servo to zero position
void resetServo(){
    dropServo.write(0);                                 // Write 0 to servo
}


// Function to read and display value of encoder 1 as a float.
// (Adapted from provided document "MD25 Circuit Guide")
float encoder1(){
    Wire.beginTransmission(MD25);                       // MD25 address
    Wire.write(ENC1);                                   // Read the position of encoder 1
    Wire.endTransmission();

    Wire.requestFrom(MD25, 4);                          // Request 4 bytes from MD25
    while(Wire.available() < 4);                        // Wait for 4 bytes to arrive
    long poss1 = Wire.read();                           // First byte for encoder 1, HH.
    poss1 <<= 8;
    poss1 += Wire.read();                               // Second byte for encoder 1, HL
    poss1 <<= 8;
    poss1 += Wire.read();                               // Third byte for encoder 1, LH
    poss1 <<= 8;
    poss1 +=Wire.read();                                // Final byte
    return(poss1*0.09);                                 // Convert to cm
}

// Function to read and display value of encoder 2 as a float
// (Adapted from provided document "MD25 Circuit Guide")
float encoder2(){
    Wire.beginTransmission(MD25);                       // MD25 address
    Wire.write(ENC2);                                   // Read the position of encoder 1
    Wire.endTransmission();

    Wire.requestFrom(MD25, 4);                          // Request 4 bytes from MD25
    while(Wire.available() < 4);                        // Wait for 4 bytes to arrive
    long poss1 = Wire.read();                           // First byte for encoder 1, HH.
    poss1 <<= 8;
    poss1 += Wire.read();                               // Second byte for encoder 1, HL
    poss1 <<= 8;
    poss1 += Wire.read();                               // Third byte for encoder 1, LH
    poss1 <<= 8;
    poss1 +=Wire.read();                                // Final byte
    return(poss1*0.09);                                 // Convert to cm
}

// Function to light an LED and sound a tone when a step has been completed
void checkpointNotify(){
    digitalWrite(ledPin, HIGH);                         // Turn on LED
    tone(piezoPin, 3000);                               // 3000Hz tone on piezo pin
    delay(30);
    tone(piezoPin, 4000);                               // 4000Hz tone on piezo pin
    delay(30);
    noTone(10);                                         // Silence piezo
    digitalWrite(ledPin, LOW);                          // Turn off LED
    delay(10);
}

// Function to move the robot forward by the amount x at speed speed
void moveForward(float x, int speed){
    do {
        Wire.beginTransmission(MD25);                   // Begin communication with MD25
        Wire.write(SPEED);                              // Write to SPEED byte
        Wire.write(speed);                              // Write speed passed to function
        Wire.endTransmission();                         // Send command, stop communication
    }while(encoder1() < x-3);                           // -3 correction factor
    stopMotors();
}

// Function to turn robot on the spot
void rotateRight(float degrees, int v){
    float wDist = (degrees/360)*robotWidth*pi;          // Calculate distance wheel must travel
    do{
        Wire.beginTransmission(MD25);                   // Begin communication with MD25
        Wire.write(TURN);                               // Write to TURN byte
        Wire.write(v);                                  // Write velocity v
        Wire.endTransmission();                         // Send command, stop communication
    }while(encoder1() < wDist-3.5);                     // -3.5 correction factor
    stopMotors();
}

// Function to turn robot on the spot
void rotateLeft(float degrees, int v){
    float wDist = (degrees/360)*robotWidth*pi;          // Calculate distance wheel must travel
    do{
        Wire.beginTransmission(MD25);                   // Begin communication with MD25
        Wire.write(TURN);                               // Write to TURN byte
        Wire.write(-v);                                 // Write velocity -v to turn opposite direction
        Wire.endTransmission();                         // Send command, stop communication
    }while(encoder2() < wDist-3.5);                     // -3.5 correction factor
    stopMotors();
}

// Function to move robot in arc
void arc(float w1d, float w2d, int v){
    float ratio = w1d/w2d;                              // Calculate velocity ratio between the two wheels
    int w1v = (int) v*ratio;                            // Declare velocity of the inner wheel
    setMode(1);                                         // Set mode 1 to control wheels independently
    do{
        Wire.beginTransmission(MD25);                   // Begin communication with MD25
        Wire.write(SPEED);                              // Write to SPEED byte (wheel 1 velocity)
        Wire.write(w1v);                                // Write calculated w1v to wheel 1 speed
        Wire.endTransmission();                         // Send command, stop communication

        Wire.beginTransmission(MD25);                   // Begin communication with MD25
        Wire.write(TURN);                               // Write to TURN byte (wheel 2 velocity)
        Wire.write(v);                                  // Write v to wheel 2 speed
        Wire.endTransmission();                         // Send command, stop communication
    }while(encoder2()<w2d-5.5);                         // -5.5 correction factor
    setMode(2);                                         // Mode 2 to use combined speed and turn values
    stopMotors();
}


// Function to stop motors
void stopMotors(){
    setAccel(6);                                        // Increase acceleration to stop quicker
    do{
        d1 = encoder2();                                // Get current encoder 2 value
        delay(5);                                       // Delay
        d2 = encoder2();                                // Get new encoder 2 value
        Wire.beginTransmission(MD25);                   // Begin communication with MD25
        Wire.write(SPEED);                              // Write to SPEED byte
        Wire.write(128);                                // Write 128 [STOP]
        Wire.endTransmission();                         // Send command, stop communication
        Wire.beginTransmission(MD25);                   // Begin communication with MD25
        Wire.write(TURN);                               // Write to TURN byte
        Wire.write(128);                                // Write 128 [STOP]
        Wire.endTransmission();                         // Send command, stop communication
    }while(d1 != d2);                                   // While the wheels are still moving
    encoderValuesReset();                               // Reset encoder values for next movement
    setAccel(2);                                        // 2 for smoother acceleration
}

// Function to change motor mode
void setMode(int mode){
    Wire.beginTransmission(MD25);                       // Begin communication with MD25
    Wire.write(MODE);                                   // Write to mode switch
    Wire.write(mode);                                   // Write mode
    Wire.endTransmission();                             // Send command, stop communication
}

void setAccel(int accel){
    Wire.beginTransmission(MD25);                       // Begin communication with MD25
    Wire.write(ACCEL);                                  // Write to mode switch
    Wire.write(accel);                                  // Write mode
    Wire.endTransmission();                             // Send command, stop communication
}
/*
------------------------------------------------------------------------------------------------------------------------
 Arduino will run this after setup.
------------------------------------------------------------------------------------------------------------------------
*/
void loop(){
    delay(500);

    //13 to 12
    moveForward(41, 165);
    checkpointNotify();

    //12 to 11
    moveForward(38, 170);
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
    moveForward(33, 200);
    dispense();
    checkpointNotify();

    //6 turn to 5
    rotateRight(90, 180);
    moveForward(34, 200);
    checkpointNotify();

    //5 turn to 4
    rotateRight(83, 180);
    moveForward(59.5, 200);
    penultimateDispense();
    checkpointNotify();

    //4 arc turn to 3
    rotateLeft(90, 180);
    arc(17.8, 60.8, 80);
    checkpointNotify();

    //3 turn to 2
    rotateRight(89, 180);
    moveForward(45, 180);
    dispense();
    checkpointNotify();

    //2 turn to 1
    rotateRight(90, 180);
    moveForward(23, 200);
    checkpointNotify();

    //1 turn to 13
    rotateLeft(90, 180);
    moveForward(29, 200);
    checkpointNotify();

    resetServo();
    delay(99999);
}
