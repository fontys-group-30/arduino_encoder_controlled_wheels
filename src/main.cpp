// #include <Arduino.h>
//
// #define enA 8
// #define stepper1 9
// #define stepper2 10
//
// #define outputA 2
// #define outputB 3
//
// int counter = 0;
// int aState;
// int aLastState;
//
// void ISR_encoder() {
//     aState = digitalRead(outputA); // Reads the "current" state of the outputA
//
//     if (counter > -240 && counter < 240) {
//         if (aState != aLastState) {
//             // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
//             if (digitalRead(outputB) != aState) {
//                 counter++;
//             } else {
//                 counter--;
//             }
//         }
//     } else {
//         digitalWrite(enA, 0);
//     }
//
//     Serial.println(counter);
//
//     aLastState = aState; // Updates the previous state of the outputA with the current state
// }
//
// void setInitialRotation() {
//     digitalWrite(stepper1, HIGH);
//     digitalWrite(stepper2, LOW);
// }
//
// void rotateCW() {
//     // Implement your clockwise rotation logic here
// }
//
// void rotateCCW() {
//     // Implement your counter-clockwise rotation logic here
// }
//
// void setup()
// {
//     Serial.begin(9600);
//     pinMode(outputA, INPUT);
//     pinMode(outputB, INPUT);
//     pinMode(enA, OUTPUT);
//     pinMode(stepper1, OUTPUT);
//     pinMode(stepper2, OUTPUT);
//
//     setInitialRotation();
//     aLastState = digitalRead(outputA);
//     digitalWrite(enA, 255);
//
//     // Attach the interrupt service routine (ISR) to the interrupt pin
//     attachInterrupt(digitalPinToInterrupt(outputA), ISR_encoder, CHANGE);  // CHANGE mode for both rising and falling edges
// }
//
// void loop() {
// }


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

void setup()
{
    Serial.begin(11520);

    front_motors.setSpeed(255);
    front_motors.forward();
}

long oldPosition  = -999;

void loop()
{
    long newPosition = front_right_encoder.read();
    if (newPosition != oldPosition) {
        oldPosition = newPosition;
        Serial.println(newPosition);
    }

    if (newPosition > 1440)
    {
        front_motors.stop();
    }

}
