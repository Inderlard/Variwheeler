# Variwheeler Library Documentation

The Variwheeler library provides the necessary classes and structures to control the motion and geometry of the Variwheeler. This documentation outlines the use of these classes and provides examples to help you get started.

## Table of Contents

- [Introduction](#introduction)
- [Classes](#classes)
  - [direction](#direction)
  - [geometry](#geometry)
- [Variables](#variables)
  - [MotorForwardDirection](#motorforwarddirection)
  - [MotorSelection](#motorselection)
  - [Single](#single)
- [Structures](#structures)
  - [Motor](#motor)
  - [Servo](#servo)
- [Examples](#examples)
  - [Using the Direction Class](#using-the-direction-class)
  - [Using the Geometry Class](#using-the-geometry-class)
- [Function Reference](#function-reference)
  - [direction class functions](#direction-class-functions)
  - [geometry class functions](#geometry-class-functions)

## Introduction

This library is designed to control the Variwheeler, a variable geometry omnidirectional car. The library includes two main classes, `direction` and `geometry`, which work together to manage the car's motion and structure.

## Classes

### direction

The `direction` class is used to control the motion of the Variwheeler. It requires the initial configuration of motor structures.

### geometry

The `geometry` class controls the geometry of the Variwheeler. It requires the initial configuration of servo structures and an instance of the `direction` class.

## Variables

### MotorForwardDirection

Defines the forward direction of the motor.

- `CW` - Defines the forward direction of the motor as clockwise.
- `CCW` - Defines the forward direction of the motor as counterclockwise.

### MotorSelection

Specifies the selection of motors.

- `A1_Motor` - Refers to the right front motor.
- `B1_Motor` - Refers to the left front motor.
- `A2_Motor` - Refers to the right rear motor.
- `B2_Motor` - Refers to the left rear motor.
- `Null_Motor`

### Single

Defines a single direction.

- `FORWARD`
- `FORWARD_RIGHT`
- `RIGHT`
- `BACKWARD_RIGHT`
- `BACKWARD`
- `BACKWARD_LEFT`
- `LEFT`
- `FORWARD_LEFT`

## Structures

### Motor

Defines the motor structure.

- `MotorForwardDirection dir` - Indicates if the motor moves forward rotating in theoretically CW or CCW.
- `uint8_t PWM_Pin_Out` - Indicates the pin of PCA9865 connected to the PWM control of the L298N.
- `uint8_t Dir1_Pin_Out` - Indicates the pin of PCA9865 connected to the Dir1 control of the L298N.
- `uint8_t Dir2_Pin_Out` - Indicates the pin of PCA9865 connected to the Dir2 control of the L298N.
- `uint16_t Max_PWM_Value = 4095` - Sets a maximum value to send PWM to this motor.
- `uint16_t Min_PWM_Value = 0` - Sets a minimum value to send PWM to this motor.
- `uint16_t Speed_Offset = 0` - Sets a correction offset to the motor in relative (PWM resolution) or real (rad/s) depending on the selected mode.

### Servo

Defines the servo structure.

- `uint8_t Pin` - Indicates the pin of PCA9865 connected to the servo.
- `uint8_t Up_End_Angle` - Sets the angle when the car is up in degrees.
- `uint8_t Down_End_Angle` - Sets the angle when the car is down in degrees.

## Examples

### Using the direction Class

To control the motion of the Variwheeler using the `direction` class, follow these steps:

```C++
#include "variwheeler.h"

// Define motor structures
Motor a1;
Motor b1;
Motor a2;
Motor b2;

// Initialize direction class
direction variwheeler(&a1, &b1, &a2, &b2);
```

```cpp
// Configuring and initializing el variwheeler

void VariwheelerConfig()
{
	// Motor configuration
  a1.dir = CW; // Direction to move motor forward
  a1.PWM_Pin_Out = 0; // pca9685 PWM pin to control the motor
  a1.Dir1_Pin_Out = 1; // pca9685 dir1 pin to control the motor
  a1.Dir2_Pin_Out = 2; // pca9685 dir2 pin to control the motor
  a1.Min_PWM_Value = 1500; // pca9685 min PWM value (over 12bit) to min torque

  b1.dir = CW; // Direction to move motor forward
  b1.PWM_Pin_Out = 3; // pca9685 PWM pin to control the motor
  b1.Dir1_Pin_Out = 4; // pca9685 dir1 pin to control the motor
  b1.Dir2_Pin_Out = 5; // pca9685 dir2 pin to control the motor
  b1.Min_PWM_Value = 1500; // pca9685 min PWM value (over 12bit) to min torque

  a2.dir = CW; // Direction to move motor forward
  a2.PWM_Pin_Out = 6; // pca9685 PWM pin to control the motor
  a2.Dir1_Pin_Out = 7; // pca9685 dir1 pin to control the motor
  a2.Dir2_Pin_Out = 8; // pca9685 dir2 pin to control the motor
  a2.Min_PWM_Value = 1500; // pca9685 min PWM value (over 12bit) to min torque

  b2.dir = CW; // Direction to move motor forward
  b2.PWM_Pin_Out = 9; // pca9685 PWM pin to control the motor
  b2.Dir1_Pin_Out = 10; // pca9685 dir1 pin to control the motor
  b2.Dir2_Pin_Out = 11; // pca9685 dir2 pin to control the motor
  b2.Min_PWM_Value = 1500; // pca9685 min PWM value (over 12bit) to min torque

  variwheeler.Config(); // Use parameters by default
}
```

```cpp
void setup()
{
  VariwheelerConfig();
}

void loop()
{
// Your code
}
```

### Using the geometry Class

To control the motion of the Variwheeler using the `direction` class, follow these steps:

```C++
#include "variwheeler.h"

// Define motor structures
Motor a1;
Motor b1;
Motor a2;
Motor b2;

// Define servo structures
Motor Sa1;
Motor Sb1;
Motor Sa2;
Motor Sb2;

// Initialize direction class
direction variwheeler(&a1, &b1, &a2, &b2);

// Initialize geometry class
direction variwheelerG(&Sa1, &Sb1, &Sa2, &Sb2, &variwheeler);
```

```cpp
// Configuring and initializing el variwheeler

void VariwheelerConfig()
{
	// Motor configuration
  a1.dir = CW; // Direction to move motor forward
  a1.PWM_Pin_Out = 0; // pca9685 PWM pin to control the motor
  a1.Dir1_Pin_Out = 1; // pca9685 dir1 pin to control the motor
  a1.Dir2_Pin_Out = 2; // pca9685 dir2 pin to control the motor
  a1.Min_PWM_Value = 1500; // pca9685 min PWM value (over 12bit) to min torque

  b1.dir = CW; // Direction to move motor forward
  b1.PWM_Pin_Out = 3; // pca9685 PWM pin to control the motor
  b1.Dir1_Pin_Out = 4; // pca9685 dir1 pin to control the motor
  b1.Dir2_Pin_Out = 5; // pca9685 dir2 pin to control the motor
  b1.Min_PWM_Value = 1500; // pca9685 min PWM value (over 12bit) to min torque

  a2.dir = CW; // Direction to move motor forward
  a2.PWM_Pin_Out = 6; // pca9685 PWM pin to control the motor
  a2.Dir1_Pin_Out = 7; // pca9685 dir1 pin to control the motor
  a2.Dir2_Pin_Out = 8; // pca9685 dir2 pin to control the motor
  a2.Min_PWM_Value = 1500; // pca9685 min PWM value (over 12bit) to min torque

  b2.dir = CW; // Direction to move motor forward
  b2.PWM_Pin_Out = 9; // pca9685 PWM pin to control the motor
  b2.Dir1_Pin_Out = 10; // pca9685 dir1 pin to control the motor
  b2.Dir2_Pin_Out = 11; // pca9685 dir2 pin to control the motor
  b2.Min_PWM_Value = 1500; // pca9685 min PWM value (over 12bit) to min torque

	Sa1.Down_End_Angle = 150; // Angle for the servo when is down
  Sa1.Up_End_Angle = 70; // Angle for the servo when is up
  Sa1.Pin = 12; // pca9685 PWM pin to control the motor

  Sb1.Down_End_Angle = 70; // Angle for the servo when is down
  Sb1.Up_End_Angle = 150; // Angle for the servo when is up
  Sb1.Pin = 13; // pca9685 PWM pin to control the motor

  Sa2.Down_End_Angle = 70; // Angle for the servo when is down
  Sa2.Up_End_Angle = 150; // Angle for the servo when is up
  Sa2.Pin = 14; // pca9685 PWM pin to control the motor

  Sb2.Down_End_Angle = 150; // Angle for the servo when is down
  Sb2.Up_End_Angle = 70; // Angle for the servo when is up
  Sb2.Pin = 15; // pca9685 PWM pin to control the motor


  variwheeler.Config(); // Use parameters by default
  variwheelerG.Config(); // Use parameters by default
}
```

```cpp
void setup()
{
  VariwheelerConfig();
}

void loop()
{
// Your code
}
```

## Function Reference

- [direction class functions](#direction-class-functions)
  - [direction](#direction)
  - [Config](#config)
  - [SetGeneralSpeed](#setgeneralspeed)
  - [SetIndividualMotorSpeed](#setindividualmotorspeed)
  - [SetSimpleDirection](#setsimpledirection)
  - [SetSimpleOrientation](#setsimpleorientation)
  - [Turn](#turn)
  - [UndefinedTurn](#undefinedturn)
  - [SetMoveVector](#setmovevector)
  - [Brake](#brake)
  - [HardBrake](#hardbrake)
  - [ReverseBrake](#reversebrake)
  - [ReverseHardBrake](#reversehardbrake)
  - [GetMotorSpeed](#getmotorspeed)
  - [GetAproximateDirection](#getaproximatedirection)
  - [GetMoveVector](#getmovevector)
- [geometry class functions](#geometry-class-functions)
  - [geometry Constructor](#geometry-constructor)
  - [Config](#config-1)
  - [SetGeometry](#setgeometry)
  - [SetGeometryFront](#setgeometryfront)
  - [SetGeometryBack](#setgeometryback)
  - [GetGeometryFront](#getgeometryfront)
  - [GetGeometryBack](#getgeometryback)

### direction class functions

#### direction

```cpp
direction(Motor *MotorA1, Motor *MotorB1, Motor *MotorA2, Motor *MotorB2)
```

Constructor of the class. Initializes the class with the necessary parameters to work, including motors and work mode.

**Parameters:**

- `MotorA1`: Pointer to the first motor struct (front right).
- `MotorB1`: Pointer to the second motor struct (front left).
- `MotorA2`: Pointer to the third motor struct (back right).
- `MotorB2`: Pointer to the fourth motor struct (back left).

**Example:**

```cpp
// Assuming Motor structures are defined elsewhere
direction variwheeler(&a1, &b1, &a2, &b2);
```

<br><br>

#### Config

```cpp
void Config(float WheelRadius, float SquaereAngleTimeOut, float AngularMaxSpeed, uint8_t MaxVoltfeed, float HorizontalSeparation = 210, float ZCompensation = 0.5, bool UseSpeedPercentaje = false, bool UseRelativeVelocity = true, bool UseDegrees = false, uint8_t PWM_Res = 12, uint8_t ReverseBrakeTime)
```

Configures technical an use format parameters to control the robot.

**Parameters:**

- `WheelRadius`: Defines the wheel radius in mm.
- `SquaereAngleTimeOut`: Defines the time it takes to rotate 90 degrees as max.
- `AngularMaxSpeed`: Defines the max angular speed of the motors.
- `MaxVoltfeed`: Max volts that motors can reach.
- `HorizontalSeparation`: Horizontal (x) separation between centers of the wheels in mm (default is 210).
- `ZCompensation`: Defines the compensation between XY and Z (default is 0.5).
- `UseSpeedPercentaje`: Defines if the speed will work in percentage format or PWM (default is false).
- `UseRelativeVelocity`: Defines if the variwheeler will use real velocity or relative (default is true).
- `UseDegrees`: Defines if the variwheeler will use degrees or radians to make turns.
- `PWM_Res`: Indicates the resolution of the PWM in bits (default is 12).
- `ReverseBrakeTime`: Indicates the time to use by reverse brake.

**Example:**

```cpp
variwheeler.Config(50.0, 2.5, 3.14, 12, 210, 0.5, true, true, true, 12, 5);

variwheeler.Config(); // Use parameters by default

```

<br><br>

#### SetGeneralSpeed

```cpp
void SetGeneralSpeed(int32_t Speed)
```

Sets the same speed to all motors and updates it.

**Parameters:**

- `Speed`: Speed to set in percentage or PWM resolution.

**Example:**

```cpp
variwheeler.SetGeneralSpeed(4095); // PWM 12 bit
variwheeler.SetGeneralSpeed(100); // Percentage
```

<br><br>

#### SetIndividualMotorSpeed

```cpp
void SetIndividualMotorSpeed(MotorSelection Mot1, int32_t Speed1, MotorSelection Mot2, int32_t Speed2, MotorSelection Mot3, int32_t Speed3)
```

Sets the speed of a motor and updates it.

**Parameters:**

- `Mot1`: Indicates which motor (A1, B1, A2, B2).
- `Speed1`: Speed to set in percentage or PWM resolution.
- `Mot2`: Indicates which motor (A1, B1, A2, B2).
- `Speed2`: Speed to set in percentage or PWM resolution.
- `Mot3`: Indicates which motor (A1, B1, A2, B2).
- `Speed3`: Speed to set in percentage or PWM resolution.
- `Mot4`: Indicates which motor (A1, B1, A2, B2).
- `Speed4`: Speed to set in percentage or PWM resolution.

**Example:**
Overload 1:

```cpp
variwheeler.SetIndividualMotorSpeed(A1_Motor, 4095); // PWM 12 bit
variwheeler.SetIndividualMotorSpeed(A1_Motor, 100);  // Percentage
variwheeler.SetIndividualMotorSpeed(A1_Motor, 5);    // Linear velocity mm/s
```

Overload 2:

```cpp
variwheeler.SetIndividualMotorSpeed(A1_Motor, 4095, B1_Motor, 4095); // PWM 12 bit
variwheeler.SetIndividualMotorSpeed(A1_Motor, 100, B1_Motor, 100);   // Percentage
variwheeler.SetIndividualMotorSpeed(A1_Motor, 5, B1_Motor, 5);       // Linear velocity mm/s
```

Overload 3:

```cpp
variwheeler.SetIndividualMotorSpeed(A1_Motor, 4095, B1_Motor, 4095, A2_Motor, 4095); // PWM 12 bit
variwheeler.SetIndividualMotorSpeed(A1_Motor, 100, B1_Motor, 100, A2_Motor, 100);    // Percentage
variwheeler.SetIndividualMotorSpeed(A1_Motor, 5, B1_Motor, 5, A2_Motor, 5);          // Linear velocity mm/s
```

Overload 4:

```cpp
variwheeler.SetIndividualMotorSpeed(A1_Motor, 4095, B1_Motor, 4095, A2_Motor, 4095, B2_Motor, 4095); // PWM 12 bit
variwheeler.SetIndividualMotorSpeed(A1_Motor, 100, B1_Motor, 100, A2_Motor, 100, B2_Motor, 100);     // Percentage
variwheeler.SetIndividualMotorSpeed(A1_Motor, 5, B1_Motor, 5, A2_Motor, 5, B2_Motor, 5);             // Linear velocity mm/s
```

<br><br>

#### SetSimpleDirection

```cpp
void SetSimpleDirection(Simple Direction)
```

Sets a scalar-like direction.

**Parameters:**

- `Direction`: Simple direction between 8 possibilities (FORWARD, FORWARD_RIGHT, RIGHT, BACKWARD_RIGHT, BACKWARD, BACKWARD_LEFT, LEFT, FORWARD_LEFT).

**Example:**

```cpp
variwheeler.SetSimpleDirection(FORWARD);
```

<br><br>

#### SetSimpleOrientation

```cpp
void SetSimpleOrientation(Simple Orientation, float RotationSpeed)
```

Sets a scalar-like orientation at a specified speed.

**Parameters:**

- `Orientation`: Simple orientation between 7 possibilities (FORWARD_RIGHT, RIGHT, BACKWARD_RIGHT, BACKWARD, BACKWARD_LEFT, LEFT, FORWARD_LEFT).
- `RotationSpeed`: Speed in percentage if `UseSpeedPercentaje = true`, PWM value if `UseRelativeVelocity = true`, speed in rad/s if both false.

**Example:**
Overload 1:

```cpp
variwheeler.SetSimpleOrientation(LEFT);
```

Overload 2:

```cpp
variwheeler.SetSimpleOrientation(RIGHT, 300.0);
```

<br><br>

#### Turn

```cpp
void Turn(float angle, int8_t Speed)
```

Turns the specified angle or radians at max speed.

**Parameters:**

- `angle`: Angle to turn in degrees if `UseDegrees = true`, else radians.
- `Speed`: Speed to make the turn.

**Example:**
Overload 1:

```cpp
variwheeler.Turn(90.0);          // degrees
variwheeler.Turn(3.1415/2); // rads
```

Overload 2:

```cpp
variwheeler.Turn(90.0, 4095); // PWM 12 bit
variwheeler.Turn(90.0, 100);  // Percentage
variwheeler.Turn(90.0, 5);      // Linear velocity
```

<br><br>

#### UndefinedTurn

```cpp
void UndefinedTurn(float RotationSpeed)
```

Turns an unspecified angle or radians at a specified speed.

**Parameters:**

- `RotationSpeed`: Speed in percentage if `UseSpeedPercentaje = true`, PWM value if `UseRelativeVelocity = true`, speed in rad/s if both false.

**Example:**
Overloard 1:

```cpp
variwheeler.UndefinedTurn();
```

Overload 2:

```cpp
variwheeler.UndefinedTurn(4095); // PWM 12 bit
variwheeler.UndefinedTurn(100);   // Percentage
variwheeler.UndefinedTurn(5);       // Linear velocity mm/s
```

<br><br>

#### SetMoveVector

```cpp
void SetMoveVector(int32_t X, int32_t Y, int32_t Z)
```

Gives the vector of movement and acts over the motors.

**Parameters:**

- `X`: Left/right movement (0 - PWM res) if relative, (0 - Max linear velocity in mm/s) if real.
- `Y`: Front/back movement (0 - PWM res) if relative, (0 - Max linear velocity in mm/s) if real.
- `Z`: CW/CCW rotation (0 - PWM res) if relative, (0 - Max linear velocity in mm/s) if real.

**Example:**

```cpp
variwheeler.SetMoveVector(1000, 1000, 500);
```

<br><br>

#### Brake

```cpp
void Brake()
```

Stops the motors by releasing them.

**Example:**

```cpp
variwheeler.Brake();
```

<br><br>

#### HardBrake

```cpp
void HardBrake()
```

Stops the motors by blocking them.

**Example:**

```cpp
variwheeler.HardBrake();
```

<br><br>

#### ReverseBrake

```cpp
void ReverseBrake()
```

Stops the motors by inverting the motors and then releasing them.

**Example:**

```cpp
variwheeler.ReverseBrake();
```

<br><br>

#### ReverseHardBrake

```cpp
void ReverseHardBrake()
```

Stops the motors by inverting the motors and then blocking them.

**Example:**

```cpp
variwheeler.ReverseHardBrake();
```

<br><br>

#### GetMotorSpeed

```cpp
int32_t GetMotorSpeed(MotorSelection Mot)
```

Gets the actual motor speed.

**Parameters:**

- `Mot`: Motor to check (A1, B1, A2, B2).

**Returns:**

- `int32_t`: Speed of the selected motor.

**Example:**

```cpp
int32_t speed = variwheeler.GetMotorSpeed(A1_Motor);
```

<br><br>

#### GetAproximateDirection

```cpp
Simple GetAproximateDirection()
```

Approximates the direction to a simple existing direction.

**Returns:**

- `Simple`: Approximated actual direction (FORWARD, FORWARD_RIGHT, RIGHT, BACKWARD_RIGHT, BACKWARD, BACKWARD_LEFT, LEFT, FORWARD_LEFT).

**Example:**

```cpp
Simple direction = variwheeler.GetAproximateDirection();
```

<br><br>

#### GetMoveVector

```cpp
void GetMoveVector(int32_t *X, int32_t *Y)
```

Gives the vector of movement.

**Parameters:**

- `X`: Pointer to left/right movement (0 - PWM res) if relative, (0 - Max linear velocity in mm/s) if real.
- `Y`: Pointer to front/back movement (0 - PWM res) if relative, (0 - Max linear velocity in mm/s) if real.

**Example:**

```cpp
int32_t x, y;
variwheeler.GetMoveVector(&x, &y);
```

### geometry class functions

#### geometry Constructor

```cpp
geometry(Servo *Servo1, Servo *Servo2, Servo *Servo3, Servo *Servo4, direction *DirClass)
```

Constructor of the class, gives the class the necessary parameters to work, including motors and work mode.

**Parameters:**

- `Servo1`: Pointer to the first motor struct, front right.
- `Servo2`: Pointer to the second motor struct, front left.
- `Servo3`: Pointer to the third motor struct, back right.
- `Servo4`: Pointer to the fourth motor struct, back left.
- `DirClass`: Pointer to the direction class.

**Preconditions:**

- It is necessary to have defined the structures of the four servos and the direction class previously.

**Example:**

```cpp
geometry variwheelerG(&servo1, &servo2, &servo3, &servo4, &variwheeler);
```

<br><br>

#### Config

```cpp
void Config(bool UseAnglePercentaje, bool DownIsReference)
```

Configurator of the class, non-mandatory.

**Parameters:**

- `UseAnglePercentaje`: Defines if the speed will work in percentage format or PWM (default is false).
- `DownIsReference`: Defines if the variwheeler will use real velocity or relative (default is true).

**Example:**

```cpp
variwheelerG.Config(true, false);
variwheelerG.Config(); // Uses default parameters
```

<br><br>

#### SetGeometry

```cpp
void SetGeometry(uint8_t Angle, uint32_t Delay)
```

Gives the vector of movement and acts over the motors.

**Parameters:**

- `Angle`: Angle of the servos in degrees or percentages (more percentage or angle = more elevated).
- `Delay`: Delay in milliseconds that it takes for the servos to reach the position.

**Example:**

Overload 1:

```cpp
variwheelerG.SetGeometry(45); // Set angle to 45 degrees or percentage
```

Overload 2:

```cpp
variwheelerG.SetGeometry(45, 1000); // Set angle to 45 degrees or percentage with a 1000 ms delay
```

<br><br>

#### SetGeometryFront

```cpp
void SetGeometryFront(uint8_t Angle, uint32_t Delay)
```

Gives the vector of movement and acts over the front motors.

**Parameters:**

- `Angle`: Angle of the servos from the front in degrees or percentages (more percentage or angle = more elevated).
- `Delay`: Delay in milliseconds that it takes for the servos to reach the position.

**Example:**

Overload 1:

```cpp
variwheelerG.SetGeometryFront(30); // Set front angle to 30 degrees or percentage
```

Overload 2:

```cpp
variwheelerG.SetGeometryFront(30, 500); // Set front angle to 30 degrees or percentage with a 500 ms delay
```

<br><br>

#### SetGeometryBack

```cpp
void SetGeometryBack(uint8_t Angle, uint32_t Delay)
```

Gives the vector of movement and acts over the back motors.

**Parameters:**

- `Angle`: Angle of the servos from the back in degrees or percentages (more percentage or angle = more elevated).
- `Delay`: Delay in milliseconds that it takes for the servos to reach the position.

**Example:**

Overload 1:

```cpp
variwheelerG.SetGeometryBack(60); // Set back angle to 60 degrees or percentage
```

Overload 2:

```cpp
variwheelerG.SetGeometryBack(60, 750); // Set back angle to 60 degrees or percentage with a 750 ms delay
```

<br><br>

#### GetGeometryFront

```cpp
uint8_t GetGeometryFront()
```

Returns the angle of the front servos in degrees or percentages.

**Return:**

- `uint8_t`: Angle of the front servos in degrees or percentages.

**Example:**

```cpp
uint8_t frontAngle = variwheelerG.GetGeometryFront(); // Get the front angle
```

<br><br>

#### GetGeometryBack

```cpp
uint8_t GetGeometryBack()
```

Returns the angle of the back servos in degrees or percentages.

**Return:**

- `uint8_t`: Angle of the back servos in degrees or percentages.

**Example:**

```cpp
uint8_t backAngle = variwheelerG.GetGeometryBack(); // Get the back angle
```

## License

This project is licensed under the GNU General Public License v3.0. See the [LICENSE](./LICENSE) file for details.
