#include <Arduino.h>
#include <L298NX2.h>
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

#define EN_A 8
#define IN1_A 9
#define IN2_A 10

#define EN_B 5
#define IN1_B 6
#define IN2_B 7

L298NX2 front_motors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);
Encoder front_right_encoder(2, 3);

// PID constants
float Kp = 3.1; // Proportional gain
float Ki = 0.01; // Integral gain
float Kd = 0.1; // Derivative gain

// PID variables
long targetPosition = 1440; // Target encoder position
long lastPosition = 0;
float integral = 0;
float previousError = 0;

// Function prototypes
void setupMotors();
void readEncoder();
void performPIDControl();
void updateMotorSpeed(float output);
bool checkTargetPosition(float error);

void setup()
{
    Serial.begin(115200);
    setupMotors(); // Initialize motors
}

long oldPosition = -999;

void loop()
{
    readEncoder(); // Read the encoder position
    performPIDControl(); // Perform PID control
    delay(10); // Small delay to prevent overload
}

void setupMotors()
{
    front_motors.stop(); // Ensure motors are stopped initially
}

void readEncoder()
{
    const long currentPosition = front_right_encoder.read();
    if (currentPosition != oldPosition)
    {
        oldPosition = currentPosition;
        Serial.println("Encoder Position: " + String(currentPosition));
    }
}

void performPIDControl()
{
    const long currentPosition = front_right_encoder.read();
    const float error = targetPosition - currentPosition;
    integral = constrain(integral + error, -1000, 1000);
    const float output = (Kp * error) + (Ki * integral) + (Kd * (error - previousError));
    previousError = error;

    if (checkTargetPosition(error))
    {
        front_motors.stop();
    }
    else
    {
        updateMotorSpeed(output);
    }
}

void updateMotorSpeed(const float output)
{
    const int motorSpeed = constrain(abs(output), 0, 255);
    if (output >= 0)
    {
        front_motors.forward();
    }
    else
    {
        front_motors.backward();
    }
    front_motors.setSpeed(motorSpeed);
}

bool checkTargetPosition(const float error)
{
    return abs(error) <= 10;
}
