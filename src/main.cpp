#define ENCODER_OPTIMIZE_INTERRUPTS

#include <Arduino.h>
#include <L298NX2.h>
#include <Encoder.h>

// Pins for L298N motor driver (front left)
#define EN_A 4
#define IN1_A 6
#define IN2_A 5

// Pins for L298N motor driver (front right)
#define EN_B 9
#define IN1_B 8
#define IN2_B 7

// Pins for L298N motor driver (back left)
#define EN_C 10
#define IN1_C 53
#define IN2_C 52

// Pins for L298N motor driver (back right)
#define EN_D 11
#define IN1_D 51
#define IN2_D 50

// Encoder pins
#define FL_ENCODER_A 3
#define FL_ENCODER_B 37

#define FR_ENCODER_A 2
#define FR_ENCODER_B 36

#define BL_ENCODER_A 19
#define BL_ENCODER_B 35

#define BR_ENCODER_A 18
#define BR_ENCODER_B 34

L298NX2 front_motors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);
L298NX2 back_motors(EN_C, IN1_C, IN2_C, EN_D, IN1_D, IN2_D);

// Encoder pins
Encoder front_left_encoder(FL_ENCODER_A, FL_ENCODER_B);
Encoder front_right_encoder(FR_ENCODER_A, FR_ENCODER_B);
Encoder back_left_encoder(BL_ENCODER_A, BL_ENCODER_B);
Encoder back_right_encoder(BR_ENCODER_A, BR_ENCODER_B);

// PID constants
float Kp = 3.1; // Proportional gain
float Ki = 0.01; // Integral gain
float Kd = 0.1; // Derivative gain

// PID variables
float integral_FL = 0, integral_FR = 0, integral_BL = 0, integral_BR = 0;
float previousError_FL = 0, previousError_FR = 0, previousError_BL = 0, previousError_BR = 0;

// Variables to track time and position for velocity calculation
long prevTime = 0;
long prevPosition_FL = 0, prevPosition_FR = 0, prevPosition_BL = 0, prevPosition_BR = 0;

// Function prototypes
void setupMotors();
void readEncoder();
void performPIDControl(float targetVelocity_FL, float targetVelocity_FR, float targetVelocity_BL, float targetVelocity_BR);
void updateMotorSpeed(float output_FL, float output_FR, float output_BL, float output_BR);
void onDataReceived(const String& data);
void EncoderValues();

void setup()
{
    Serial.begin(115200);
    setupMotors(); // Initialize motors
}


void sendEncoderValues();

void loop()
{
    static unsigned long lastSendTime = 0;
    const unsigned long currentTime = millis();

    if (currentTime - lastSendTime >= 500) // Send every half second
    {
        sendEncoderValues();
        lastSendTime = currentTime;
    }

    if (Serial.available())
    {
        const String data = Serial.readStringUntil('\n');
        onDataReceived(data);
    }
}

void sendEncoderValues()
{
    const long currentTime = millis();
    const float deltaTime = (currentTime - prevTime) / 1000.0;

    // Read current encoder positions
    const long currentPosition_FL = front_left_encoder.read();
    const long currentPosition_FR = front_right_encoder.read();
    const long currentPosition_BL = back_left_encoder.read();
    const long currentPosition_BR = back_right_encoder.read();

    // Calculate velocities
    const float velocity_FL = (currentPosition_FL - prevPosition_FL) / deltaTime;
    const float velocity_FR = (currentPosition_FR - prevPosition_FR) / deltaTime;
    const float velocity_BL = (currentPosition_BL - prevPosition_BL) / deltaTime;
    const float velocity_BR = (currentPosition_BR - prevPosition_BR) / deltaTime;

    // Format the values as a CSV string
    const String message = "OUT: " + String(velocity_FL) + "," + String(velocity_FR * -1) + "," + String(velocity_BL) + "," + String(velocity_BR * -1) + "\n";

    // Convert the message to a UTF-8 encoded string
    const char* utf8EncodedMessage = message.c_str();

    // Send the UTF-8 encoded string over serial
    Serial.print(utf8EncodedMessage);

    // Update previous positions and time
    prevPosition_FL = currentPosition_FL;
    prevPosition_FR = currentPosition_FR;
    prevPosition_BL = currentPosition_BL;
    prevPosition_BR = currentPosition_BR;
    prevTime = currentTime;
}

void setupMotors()
{
    front_motors.stop();
    back_motors.stop();
}

void performPIDControl(const float targetVelocity_FL, const float targetVelocity_FR, const float targetVelocity_BL, const float targetVelocity_BR)
{
    const long currentTime = millis();
    const float deltaTime = (currentTime - prevTime) / 1000.0;

    // Read current encoder positions
    const long currentPosition_FL = front_left_encoder.read();
    const long currentPosition_FR = front_right_encoder.read();
    const long currentPosition_BL = back_left_encoder.read();
    const long currentPosition_BR = back_right_encoder.read();

    const float velocity_FL = (currentPosition_FL - prevPosition_FL) / deltaTime;
    const float velocity_FR = (currentPosition_FR - prevPosition_FR) / deltaTime;
    const float velocity_BL = (currentPosition_BL - prevPosition_BL) / deltaTime;
    const float velocity_BR = (currentPosition_BR - prevPosition_BR) / deltaTime;

    // Calculate errors
    const float error_FL = targetVelocity_FL - velocity_FL;
    const float error_FR = targetVelocity_FR - velocity_FR;
    const float error_BL = targetVelocity_BL - velocity_BL;
    const float error_BR = targetVelocity_BR - velocity_BR;

    // Update integrals
    integral_FL += error_FL * deltaTime;
    integral_FR += error_FR * deltaTime;
    integral_BL += error_BL * deltaTime;
    integral_BR += error_BR * deltaTime;

    // Prevent integral windup
    integral_FL = constrain(integral_FL, -1000, 1000);
    integral_FR = constrain(integral_FR, -1000, 1000);
    integral_BL = constrain(integral_BL, -1000, 1000);
    integral_BR = constrain(integral_BR, -1000, 1000);

    // Calculate derivatives
    const float derivative_FL = (error_FL - previousError_FL) / deltaTime;
    const float derivative_FR = (error_FR - previousError_FR) / deltaTime;
    const float derivative_BL = (error_BL - previousError_BL) / deltaTime;
    const float derivative_BR = (error_BR - previousError_BR) / deltaTime;

    // Calculate outputs
    const float output_FL = (Kp * error_FL) + (Ki * integral_FL) + (Kd * derivative_FL);
    const float output_FR = (Kp * error_FR) + (Ki * integral_FR) + (Kd * derivative_FR);
    const float output_BL = (Kp * error_BL) + (Ki * integral_BL) + (Kd * derivative_BL);
    const float output_BR = (Kp * error_BR) + (Ki * integral_BR) + (Kd * derivative_BR);

    // Update motor speeds
    updateMotorSpeed(output_FL, output_FR, output_BL, output_BR);

    // Stop motors if target velocity is 0
    if (targetVelocity_FL == 0) front_motors.stopA();
    if (targetVelocity_FR == 0) front_motors.stopB();
    if (targetVelocity_BL == 0) back_motors.stopA();
    if (targetVelocity_BR == 0) back_motors.stopB();

    // Store current errors and positions for next loop
    previousError_FL = error_FL;
    previousError_FR = error_FR;
    previousError_BL = error_BL;
    previousError_BR = error_BR;

    prevPosition_FL = currentPosition_FL;
    prevPosition_FR = currentPosition_FR;
    prevPosition_BL = currentPosition_BL;
    prevPosition_BR = currentPosition_BR;
    prevTime = currentTime;
}

void updateMotorSpeed(const float output_FL, const float output_FR, const float output_BL, const float output_BR)
{
    front_motors.setSpeedA(constrain(abs(output_FL), 100, 255));
    front_motors.setSpeedB(constrain(abs(output_FR), 100, 255));
    back_motors.setSpeedA(constrain(abs(output_BL), 100, 255));
    back_motors.setSpeedB(constrain(abs(output_BR), 100, 255));

    output_FL >= 0 ? front_motors.forwardA() : front_motors.backwardA();
    output_FR >= 0 ? front_motors.forwardB() : front_motors.backwardB();
    output_BL >= 0 ? back_motors.forwardA() : back_motors.backwardA();
    output_BR >= 0 ? back_motors.forwardB() : back_motors.backwardB();
}

void onDataReceived(const String& data)
{
    if (!data.startsWith("IN: ")) return;

    const String csvData = data.substring(3);

    const int commaIndex1 = csvData.indexOf(',');
    const int commaIndex2 = csvData.indexOf(',', commaIndex1 + 1);
    const int commaIndex3 = csvData.indexOf(',', commaIndex2 + 1);

    if (commaIndex1 == -1 || commaIndex2 == -1 || commaIndex3 == -1) return;

    const float targetVelocity_FL = csvData.substring(0, commaIndex1).toFloat();
    const float targetVelocity_FR = csvData.substring(commaIndex1 + 1, commaIndex2).toFloat();
    const float targetVelocity_BL = csvData.substring(commaIndex2 + 1, commaIndex3).toFloat();
    const float targetVelocity_BR = csvData.substring(commaIndex3 + 1).toFloat();

    performPIDControl(targetVelocity_FL, targetVelocity_FR, targetVelocity_BL, targetVelocity_BR);
}
