#define ENCODER_OPTIMIZE_INTERRUPTS

#include <Arduino.h>
#include <L298NX2.h>
#include <Encoder.h>
#include <ArduinoJson.h>

// Pins for L298N motor driver (front left)
#define EN_A 7
#define IN1_A 8
#define IN2_A 9

// Pins for L298N motor driver (front right)
#define EN_B 4
#define IN1_B 5
#define IN2_B 6

// Pins for L298N motor driver (back left)
#define EN_C 10
#define IN1_C 52
#define IN2_C 53

// Pins for L298N motor driver (back right)
#define EN_D 11
#define IN1_D 50
#define IN2_D 51

L298NX2 front_motors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);
L298NX2 back_motors(EN_C, IN1_C, IN2_C, EN_D, IN1_D, IN2_D);

// Encoder pins
Encoder front_right_encoder(2, 36);
Encoder front_left_encoder(3, 37);
Encoder back_right_encoder(18, 34);
Encoder back_left_encoder(19, 35);

// PID constants
float Kp = 3.1; // Proportional gain
float Ki = 0.01; // Integral gain
float Kd = 0.1; // Derivative gain

// PID variables
float integral_FL = 0, integral_FR = 0, integral_BL = 0, integral_BR = 0;
float previousError_FL = 0, previousError_FR = 0, previousError_BL = 0, previousError_BR = 0;

// Function prototypes
void setupMotors();
void readEncoder();
void performPIDControl(float targetVelocity_FL, float targetVelocity_FR, float targetVelocity_BL, float targetVelocity_BR);
void updateMotorSpeed(float output_FL, float output_FR, float output_BL, float output_BR);
void onDataReceived(const String& data);

void setup()
{
    Serial.begin(115200);
    setupMotors(); // Initialize motors
}

void loop()
{
    if (Serial.available())
    {
        const String data = Serial.readStringUntil('\n');
        onDataReceived(data);
    }
    delay(10); // Small delay to prevent overload
}

void setupMotors()
{
    front_motors.stop();
    back_motors.stop();
}

void performPIDControl(const float targetVelocity_FL, const float targetVelocity_FR, const float targetVelocity_BL, const float targetVelocity_BR)
{
    // Read current encoder position
    const long currentPosition = front_right_encoder.read();

    // Calculate errors
    const float error_FL = targetVelocity_FL - static_cast<float>(currentPosition);
    const float error_FR = targetVelocity_FR - static_cast<float>(currentPosition);
    const float error_BL = targetVelocity_BL - static_cast<float>(currentPosition);
    const float error_BR = targetVelocity_BR - static_cast<float>(currentPosition);

    // Update integrals
    integral_FL += error_FL;
    integral_FR += error_FR;
    integral_BL += error_BL;
    integral_BR += error_BR;

    // Prevent integral windup
    integral_FL = constrain(integral_FL, -1000, 1000);
    integral_FR = constrain(integral_FR, -1000, 1000);
    integral_BL = constrain(integral_BL, -1000, 1000);
    integral_BR = constrain(integral_BR, -1000, 1000);

    // Calculate derivatives
    const float derivative_FL = error_FL - previousError_FL;
    const float derivative_FR = error_FR - previousError_FR;
    const float derivative_BL = error_BL - previousError_BL;
    const float derivative_BR = error_BR - previousError_BR;

    // Calculate outputs
    const float output_FL = (Kp * error_FL) + (Ki * integral_FL) + (Kd * derivative_FL);
    const float output_FR = (Kp * error_FR) + (Ki * integral_FR) + (Kd * derivative_FR);
    const float output_BL = (Kp * error_BL) + (Ki * integral_BL) + (Kd * derivative_BL);
    const float output_BR = (Kp * error_BR) + (Ki * integral_BR) + (Kd * derivative_BR);

    // Update motor speeds
    updateMotorSpeed(output_FL, output_FR, output_BL, output_BR);

    // Store current errors for next loop
    previousError_FL = error_FL;
    previousError_FR = error_FR;
    previousError_BL = error_BL;
    previousError_BR = error_BR;
}

void updateMotorSpeed(const float output_FL, const float output_FR, const float output_BL, const float output_BR)
{
    front_motors.setSpeedA(constrain(abs(output_FL), 0, 255));
    front_motors.setSpeedB(constrain(abs(output_FR), 0, 255));
    back_motors.setSpeedA(constrain(abs(output_BL), 0, 255));
    back_motors.setSpeedB(constrain(abs(output_BR), 0, 255));

    output_FL >= 0 ? front_motors.forwardA() : front_motors.backwardA();
    output_FR >= 0 ? front_motors.forwardB() : front_motors.backwardB();
    output_BL >= 0 ? back_motors.forwardA() : back_motors.backwardA();
    output_BR >= 0 ? back_motors.forwardB() : back_motors.backwardB();
}

void onDataReceived(const String& data)
{
    JsonDocument doc;

    if (deserializeJson(doc, data)) {
        Serial.println(F("deserializeJson() failed"));
        return;
    }

    performPIDControl(doc["wheel_FL"], doc["wheel_FR"], doc["wheel_BL"], doc["wheel_BR"]);
}