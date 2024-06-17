// ============================================================================
// Biblioteca de control vertorial y geometrico de el variwheeler
// ============================================================================

// Versión 1.0.0 (2024-05-31)
// Desarrollado por: Javier Zanon Saiz (Alias Inderlard)
// GitHub: https://github.com/Inderlard/Variwheeler
// Licencia: GNU General Public License v3.0

// Descripción breve:
// Esta biblioteca proporciona dos clases complementarias, direccion y geometria
// estas clases estan destinadas al control de los motores principales y los
// servos, respectivamente, ademas, la clase de geometria hace uso de la de
// direccion para asegurar el apoyo de los servos y evitar forzarlos.

#ifndef VARIWHEELER_H
#define VARIWHEELER_H

#include "Arduino.h"
#include <Wire.h>
#include "Adafruit_PWMServoDriver.h"
#include <math.h>

#define LOGIC_ONE 4095, 0
#define LOGIC_ZERO 0, 4095
#define Degree_To_Pulse_Factor 2.183333333
#define Degree_To_Pulse_Offset 172
// (pow(2, PWM_Resolution)-1) / ((sqrt(2) * (pow(2, PWM_Resolution)-1) / 2) + pow(2, PWM_Resolution)-1);

enum MotorForwardDirection
{
    CW, // Defines the forward direction of the motor as CW
    CCW // Defines the forward direction of the motor as CCW
};

enum MotorSelection
{
    A1_Motor, // Refers to the front right motor
    B1_Motor, // Refers to the front left motor
    A2_Motor, // Refers to the back right motor
    B2_Motor, // Refers to the back left motor
    Null_Motor

};

enum Simple
{
    FORWARD,
    FORWARD_RIGHT,
    RIGHT,
    BACKWARD_RIGHT,
    BACKWARD,
    BACKWARD_LEFT,
    LEFT,
    FORWARD_LEFT

};

/*enum SimpleOrientation
{
    FORWARD_RIGHT,
    RIGHT,
    BACKWARD_RIGHT,
    BACKWARD,
    BACKWARD_LEFT,
    LEFT,
    FORWARD_LEFT
};*/

struct Motor
{
    MotorForwardDirection dir;     // Indicates if the motor moves forward rotating in theorically CW or CCW, used to invert the nominal direction if needed
    uint8_t PWM_Pin_Out;           // Indicates the pin of PCA9865 is connected the PWM control of the L298N
    uint8_t Dir1_Pin_Out;          // Indicates the pin of PCA9865 is connected the Dir1 control of the L298N
    uint8_t Dir2_Pin_Out;          // Indicates the pin of PCA9865 is connected the Dir2 control of the L298N
    uint16_t Max_PWM_Value = 4095; // Set a maximum value to send PWM to this motor
    uint16_t Min_PWM_Value = 0;    // Set a minimum value to send PWM to this motor
    uint16_t Speed_Offset = 0;     // Set a correction Offset to the motor in relative (PWM resolution) or real (rad/s) depending of the selected mode
};

struct Servo
{
    uint8_t Pin;            // Indicates the pin of PCA9865 is connected the servo
    uint8_t Up_End_Angle;   // Set the angle when the car is up in degrees
    uint8_t Down_End_Angle; // Set the angle when the car is down in degrees
};

class direction
{
public:
    direction(Motor *MotorA1, Motor *MotorB1, Motor *MotorA2, Motor *MotorB2); // Constructor

    void Config(
        float WheelRadius = 300,           // Wheel radius
        float SquaereAngleTimeOut = 600,   // Estimated time that needs to turn 90 degrees at maximum power (in ms)
        float AngularMaxSpeed = 23.143066, // Angular speed for TT motor at 8V in rad/s (Only needed if Relative_Velocity is false) (With no load 290 RPM with load estimated 221 RPM)
        uint8_t MaxVoltfeed = 8,           // Max volts that motors can reach
        float HorizontalSeparation = 210,  // Horizontal (x) separation between centers of the wheels (in mm)
        float ZCompensation = 0.3,         // Defines the compensation between XY and Z, 0.3 by default (Higher value gives more priority to Z over XY)
        bool UseSpeedPercentaje = false,   // Defines if the speed will work in percentage format or PWM (ture indicates percentaje) (false by default)
        bool UseRelativeVelocity = true,   // Defines if the variwheeler will use real velocity or relative (relative means no control over specific velocity) (true indicates relative) (true by default)
        bool UseDegrees = true,            // Defines if the variwheeler will use degrees or radians to make turns
        uint8_t PWM_Res = 12,              // Indicate the resolution of the PWM in bits (12 by default)
        uint8_t ReverseBrakeTime = 10);    // Indicate the time to use by reverse brake (10 by default)

    void
    SetGeneralSpeed(int32_t Speed);                                                                                                                                                   // Set all motors speed (if percentaje enabled, use conversion first)
    void SetIndividualMotorSpeed(MotorSelection Mot, int32_t Speed);                                                                                                                  // Set only one motor speed (if percentaje enabled, use conversion first)
    void SetIndividualMotorSpeed(MotorSelection Mot1, int32_t Speed1, MotorSelection Mot2, int32_t Speed2);                                                                           // Set two motor speed (if percentaje enabled, use conversion first)
    void SetIndividualMotorSpeed(MotorSelection Mot1, int32_t Speed1, MotorSelection Mot2, int32_t Speed2, MotorSelection Mot3, int32_t Speed3);                                      // Set three motor speed (if percentaje enabled, use conversion first)
    void SetIndividualMotorSpeed(MotorSelection Mot1, int32_t Speed1, MotorSelection Mot2, int32_t Speed2, MotorSelection Mot3, int32_t Speed3, MotorSelection Mot4, int32_t Speed4); // Set fourth motor speed (if percentaje enabled, use conversion first)
    void SetSimpleDirection(Simple Direction);                                                                                                                                        // Determinates from "scalar mode" a direction movement from 8 options
    void SetSimpleOrientation(Simple Orientation, float RotationSpeed);                                                                                                               // Determinates from "scalar mode" a orientation movement from 7 options at specified speed
    void SetSimpleOrientation(Simple Orientation);                                                                                                                                    // Determinates from "scalar mode" a orientation movement from 7 options
    void Turn(float angle, float RotationSpeed);                                                                                                                                      // Rotate an angle in degrees or radians at specified speed
    void Turn(float angle);                                                                                                                                                           // Rotate an angle in degrees or radians
    void UndefinedTurn(float RotationSpeed);                                                                                                                                          // Rotate an angle in degrees or radians at specified speed
    void UndefinedTurn();                                                                                                                                                             // Rotate an angle in degrees or radians
    void SetMoveVector(int32_t X, int32_t Y, int32_t Z);                                                                                                                              // Determinates the speed and the direction in function of value of the descomposition XY of the vector and Z for rotation
    void Brake();                                                                                                                                                                     // Stops the motors by releasing the motors
    void HardBrake();                                                                                                                                                                 // Stops the motors by blocking the motors
    void ReverseBrake();                                                                                                                                                              // Stops the motors by inverting the motors and then releasing
    void ReverseHardBrake();                                                                                                                                                          // Stops the motors by inverting the motors and then blocking

    int32_t GetMotorSpeed(MotorSelection Mot);  // Get the value of the speed of the selected motor
    Simple GetAproximateDirection();            // Get the value of the "scalar mode" direction
    void GetMoveVector(int32_t *X, int32_t *Y); // Get the value of the vector

    Adafruit_PWMServoDriver *ReturnAdafruit_PWMServoDriver();
    Adafruit_PWMServoDriver variwheelerControl;

private:
    int32_t X_Vector = 0,
            Y_Vector = 0, Z_Vector = 0;
    float A1_Speed = 0, B1_Speed = 0, A2_Speed = 0, B2_Speed = 0, GeneralSpeed = 0;
    float A1_Vector = 0, B1_Vector = 0, A2_Vector = 0, B2_Vector = 0; // Variables vectoriales de los motors
    Motor *Motor_A1;
    Motor *Motor_B1;
    Motor *Motor_A2;
    Motor *Motor_B2;

    // Configuration parameters:
    float Wheel_Radius = 300;            // Wheel radius in mm
    float Squaere_Angle_Time_Out = 600;  // Estimated time that needs to turn 90 degrees at maximum power (in ms)
    float Angular_Max_Speed = 23.143066; // Angular speed for TT motor at 8V in rad/s (Only needed if Relative_Velocity is false) (With no load 290 RPM with load estimated 221 RPM)
    uint8_t Max_Volt_feed = 8;
    float Horizontal_Separation = 210;
    float Compensator = 0.3;
    bool Use_Speed_Percentaje;
    bool Relative_Velocity;
    bool Use_Degrees;
    uint8_t PWM_Resolution = 12;
    uint8_t Reverse_Brake_Time = 10;
    float Reescaler_math = 0.82842;
    float Reescaler_PWM = 1;
    float Reescaler_Volt = 0.3456758927;
    float Volt_to_PWM = 512;
};

class geometry
{
public:
    geometry(Servo *Servo1, Servo *Servo2, Servo *Servo3, Servo *Servo4, direction *DirClass); // Constructor
    void Config(bool UseAnglePercentaje = true, bool DownIsReference = true);                  // Configuration of the servos
    void SetGeometry(uint8_t Angle);                                                           // Set the angle of servos to variate the heigh at max velocity
    void SetGeometry(uint8_t Angle, uint32_t Delay);                                           // Set the angle of servos to variate the heigh at specific velocity
    void SetGeometryFront(uint8_t Angle);                                                      // Set the angle of servos in the front to variate the heigh at max velocity
    void SetGeometryFront(uint8_t Angle, uint32_t Delay);                                      // Set the angle of servos in the front to variate the heigh at specific velocity
    void SetGeometryBack(uint8_t Angle);                                                       // Set the angle of servos in the back to variate the heigh at max velocity
    void SetGeometryBack(uint8_t Angle, uint32_t Delay);                                       // Set the angle of servos in the back to variate the heigh at specific velocity

    uint8_t GetGeometryFront(); // Returns the actual angle of the front
    uint8_t GetGeometryBack();  // Returns the actual angle of the back

private:
    uint8_t Servos_Angle_Front = 0, Servos_Angle_Back = 0, Restringed_Front_travel, Restringed_Back_travel;
    bool Use_Angle_Percentaje = true, Down_Is_Reference = true;
    uint8_t Servo_1_RealAngle, Servo_2_RealAngle, Servo_3_RealAngle, Servo_4_RealAngle;
    Servo *Servo_1;
    Servo *Servo_2;
    Servo *Servo_3;
    Servo *Servo_4;
    direction *Compresion; // Instance of the direction object
    Adafruit_PWMServoDriver *variwheelerControl;
};

#endif