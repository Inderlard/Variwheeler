#include "variwheeler.h"

/*
  |     |
_        _
     *                  *

  |     |
   _   _


   y
      x


v1=y+x-z  ((1-P)*sqr(2)*(y+x)/2-P*z)
v2=y-x+z  ((1-P)*sqr(2)*(y-x)/2+P*z)
v3=y-x-z  ((1-P)*sqr(2)*(y+x)/2+P*z)
v4=y+x+z  ((1-P)*sqr(2)*(y-x)/2-P*z)

! AÑADIR A LA CONFIGURACION:
Compensator: Es la forma en la que se ajusta la relacion entre la rotacion en Z y el movimiento lineal

? No se configuran pero se calculan a partir de la configuracion
Reescaler_Math: Reescaler_math sirve para reescalar la escala de salida de las ecuaciones de movimiento a la escala del PWM
Reescaler_PWM: Reescaler_PWM pasa de la resolucion a los 12 bits



vi es la velocidad LINEAL a la que se mueve la rueda (NO EL ROBOT) El robot se movera a una velocidad diferente en realidad (4*y, 4*x)
Esto debe ser pasado a angular y luego a Voltios para finalmente a PWM y este es el valor a escribir.

En el calculo debe tomarse en cuenta si una rueda es 0 en la resultante, puesto que entonces se pondera en funcion siendo la velocidad final x (o y) * Nruedas en funcionamiento
*/

/**
 * @brief Constructor of the class, give to the class the necessary params to work, motors, and work mode
 *
 * @param MotorA1 Motor struct pointer. Pointer to the first motor struct, front right.
 * @param MotorB1 Motor struct pointer. Pointer to the second motor struct, front left.
 * @param MotorA2 Motor struct pointer. Pointer to the third motor struct, back right.
 * @param MotorB2 Motor struct pointer. Pointer to the fourth motor struct, back left.
 *
 * @pre Is neccesary to have defined the structures of the fourth motors previusly
 */
direction::direction(Motor *MotorA1, Motor *MotorB1, Motor *MotorA2, Motor *MotorB2)
{
  Motor_A1 = MotorA1;
  Motor_B1 = MotorB1;
  Motor_A2 = MotorA2;
  Motor_B2 = MotorB2;
  variwheelerControl = Adafruit_PWMServoDriver(0x40);
}

/**
 * @brief Configure technical parameters to control the robot
 *
 * @param WheelRadius float. Defines the wheel radius in mm
 * @param SquaereAngleTimeOut float. Defines the time it takes to rotate 90 as max
 * @param AngularMaxSpeed float. Defines the max angular speed of the motors
 * @param Max_Volt_feed uint8_t. Max volts that motors can reach
 * @param HorizontalSeparation float. Horizontal (x) separation between centers of the wheels (in mm) (210mm by default)
 * @param ZCompensation float. Defines the compensation between XY and Z, 0.5 by default (Higher value gives more priority to Z over XY)
 * @param UseSpeedPercentaje boolean. Defines if the speed will work in percentage format or PWM (ture indicates percentaje) (false by default)
 * @param UseRelativeVelocity boolean. Defines if the variwheeler will use real velocity or relative (relative means no control over specific velocity) (true indicates relative) (true by default)
 * @param UseDegrees boolean. Defines if the variwheeler will use degrees or radians to make turns
 * @param PWM_Res uint8_t. Indicate the resolution of the PWM in bits (12 by default)
 * @param ReverseBrakeTime uint8_t. Indicate the time to use by reverse brake
 *
 * @retval None
 */
void direction::Config(float WheelRadius, float SquaereAngleTimeOut, float AngularMaxSpeed, uint8_t MaxVoltfeed, float HorizontalSeparation, float ZCompensation, bool UseSpeedPercentaje, bool UseRelativeVelocity, bool UseDegrees, uint8_t PWM_Res, uint8_t ReverseBrakeTime)
{
  // Integracion de parametros de movimiento
  Wheel_Radius = WheelRadius;                   // Wheel radius in mm
  Squaere_Angle_Time_Out = SquaereAngleTimeOut; // Estimated time that needs to turn 90 degrees at maximum power (in ms)
  Angular_Max_Speed = AngularMaxSpeed;          // Angular speed for TT motor at 8V in rad/s (Only needed if Relative_Velocity is false) (With no load 290 RPM with load estimated 221 RPM)
  Max_Volt_feed = MaxVoltfeed;
  HorizontalSeparation = Horizontal_Separation,
  Compensator = ZCompensation;

  // Integracion de parametros de control
  Use_Speed_Percentaje = UseSpeedPercentaje;
  Relative_Velocity = UseRelativeVelocity;
  PWM_Resolution = PWM_Res;
  Use_Degrees = UseDegrees;
  Reverse_Brake_Time = ReverseBrakeTime;

  // Calculos de los parametros internos
  Reescaler_math = (pow(2, PWM_Resolution) - 1) / ((1 - Compensator) * sqrt(2) * (pow(2, PWM_Resolution) - 1) + Compensator * (pow(2, PWM_Resolution) - 1));
  Reescaler_PWM = 4095 / (pow(2, PWM_Resolution) - 1);

  /*
  |                 |               |               |               |               |
  | ---             | ---           | ---           | ---           | ---           |
  |                 | 3V            | 5V            | 6V            | 8V            |
  | No-load speed   | 13.090 rad/s  | 20.9440 rad/s | 24.0856 rad/s | 30.3688 rad/s |
  | Speed with load | 9.9484 rad/s  | 15.9174 rad/s | 18.3260 rad/s | 23.1431 rad/s | Using with load scale by default
  */
  Reescaler_Volt = MaxVoltfeed / AngularMaxSpeed; // rad/s to volts

  Volt_to_PWM = 4095 / MaxVoltfeed;

  if (!Relative_Velocity)
  {
    Use_Speed_Percentaje = false;
  }

  // Inicializacion del PCA9685
  variwheelerControl.begin();
  variwheelerControl.setPWMFreq(60); // Frecuecia PWM de 60Hz o T=16,66ms

#ifdef DebuggingEnable
  OnlineSerial.ServerBegin();
#endif
}

/**
 * @brief Sets the same speed to all motors and update it.
 *
 * @param Speed int32_t ± (2^PWM_Resolution)-1. Speed to set in percetage or PWM res
 *
 * @retval None
 */
void direction::SetGeneralSpeed(int32_t Speed)
{
  SetIndividualMotorSpeed(A1_Motor, Speed, B1_Motor, Speed, A2_Motor, Speed, B2_Motor, Speed);
}

/**
 * @brief Sets the speed of a motor and update it
 *
 * @param Mot MotorSelection. Indicates wich motor (A1, B1, A2, B2)
 * @param Speed int32_t ± (2^PWM_Resolution)-1. Speed to set in percetage, PWM res or linear velocity
 *
 * @retval None
 */
void direction::SetIndividualMotorSpeed(MotorSelection Mot, int32_t Speed)
{
  if (abs(Speed) <= pow(2, PWM_Resolution) - 1)
  {
    if ((Speed >= 100 || Speed <= -100) && Use_Speed_Percentaje)
    {
      // TODO: En esta version todavia no se reportan errores
      return;
    }

    int32_t OriginalSpeed = Speed;
    if (Use_Speed_Percentaje)
    {
      Speed = (pow(2, PWM_Resolution) - 1) * Speed / 100;
    }
    else if (!Relative_Velocity)
    {
      Speed = abs(Speed / Wheel_Radius); // Paso a velocidad angular y posteriormente a PWM v=w*r
      if (Speed > Angular_Max_Speed)
      {
        // TODO: En esta version todavia no se reportan errores
        return;
      }
      else
      {
        Speed = Speed * Reescaler_Volt * Volt_to_PWM;
      }
    }

    switch (Mot)
    {
    case A1_Motor:
      A1_Speed = OriginalSpeed;
      A1_Vector = A1_Speed;

      // Direccionamiento
      if ((A1_Speed < 0 && Motor_A1->dir == CW) || (A1_Speed > 0 && Motor_A1->dir == CCW))
      {
        variwheelerControl.setPWM(Motor_A1->Dir2_Pin_Out, LOGIC_ZERO); // 1 Logico
        variwheelerControl.setPWM(Motor_A1->Dir1_Pin_Out, LOGIC_ONE);  // 0 logico
      }
      else if ((A1_Speed > 0 && Motor_A1->dir == CW) || (A1_Speed < 0 && Motor_A1->dir == CCW))
      {
        variwheelerControl.setPWM(Motor_A1->Dir1_Pin_Out, LOGIC_ZERO); // 0 logico
        variwheelerControl.setPWM(Motor_A1->Dir2_Pin_Out, LOGIC_ONE);  // 1 Logico
      }

      // PWM
      if (abs(Speed) > Motor_A1->Max_PWM_Value)
        Speed = Motor_A1->Max_PWM_Value;
      else if (abs(Speed) < Motor_A1->Min_PWM_Value && abs(Speed) != 0)
        Speed = Motor_A1->Min_PWM_Value;
      variwheelerControl.setPWM(Motor_A1->PWM_Pin_Out, 1, abs(Speed));
      break;

    case B1_Motor:
      B1_Speed = OriginalSpeed;
      B1_Vector = B1_Speed;

      // Direccionamiento
      if ((B1_Speed < 0 && Motor_B1->dir == CW) || (B1_Speed > 0 && Motor_B1->dir == CCW))
      {
        variwheelerControl.setPWM(Motor_B1->Dir2_Pin_Out, LOGIC_ZERO); // 1 Logico
        variwheelerControl.setPWM(Motor_B1->Dir1_Pin_Out, LOGIC_ONE);  // 0 logico
      }
      else if ((B1_Speed > 0 && Motor_B1->dir == CW) || (B1_Speed < 0 && Motor_B1->dir == CCW))
      {
        variwheelerControl.setPWM(Motor_B1->Dir1_Pin_Out, LOGIC_ZERO); // 0 logico
        variwheelerControl.setPWM(Motor_B1->Dir2_Pin_Out, LOGIC_ONE);  // 1 Logico
      }

      // PWM
      if (abs(Speed) > Motor_B1->Max_PWM_Value)
        Speed = Motor_B1->Max_PWM_Value;
      else if (abs(Speed) < Motor_B1->Min_PWM_Value && abs(Speed) != 0)
        Speed = Motor_B1->Min_PWM_Value;
      variwheelerControl.setPWM(Motor_B1->PWM_Pin_Out, 1, abs(Speed));
      break;

    case A2_Motor:
      A2_Speed = OriginalSpeed;
      A2_Vector = A2_Speed;

      // Direccionamiento
      if ((A2_Speed < 0 && Motor_A2->dir == CW) || (A2_Speed > 0 && Motor_A2->dir == CCW))
      {
        variwheelerControl.setPWM(Motor_A2->Dir2_Pin_Out, LOGIC_ZERO); // 1 Logico
        variwheelerControl.setPWM(Motor_A2->Dir1_Pin_Out, LOGIC_ONE);  // 0 logico
      }
      else if ((A2_Speed > 0 && Motor_A2->dir == CW) || (A2_Speed < 0 && Motor_A2->dir == CCW))
      {
        variwheelerControl.setPWM(Motor_A2->Dir1_Pin_Out, LOGIC_ZERO); // 0 logico
        variwheelerControl.setPWM(Motor_A2->Dir2_Pin_Out, LOGIC_ONE);  // 1 Logico
      }

      // PWM
      if (abs(Speed) > Motor_A2->Max_PWM_Value)
        Speed = Motor_A2->Max_PWM_Value;
      else if (abs(Speed) < Motor_A2->Min_PWM_Value && abs(Speed) != 0)
        Speed = Motor_A2->Min_PWM_Value;
      variwheelerControl.setPWM(Motor_A2->PWM_Pin_Out, 1, abs(Speed));
      break;

    case B2_Motor:
      B2_Speed = OriginalSpeed;
      B2_Vector = B2_Speed;

      // Direccionamiento
      if ((B2_Speed < 0 && Motor_B2->dir == CW) || (B2_Speed > 0 && Motor_B2->dir == CCW))
      {
        variwheelerControl.setPWM(Motor_B2->Dir2_Pin_Out, LOGIC_ZERO); // 1 Logico
        variwheelerControl.setPWM(Motor_B2->Dir1_Pin_Out, LOGIC_ONE);  // 0 logico
      }
      else if ((B2_Speed > 0 && Motor_B2->dir == CW) || (B2_Speed < 0 && Motor_B2->dir == CCW))
      {
        variwheelerControl.setPWM(Motor_B2->Dir1_Pin_Out, LOGIC_ZERO); // 0 logico
        variwheelerControl.setPWM(Motor_B2->Dir2_Pin_Out, LOGIC_ONE);  // 1 Logico
      }

      // PWM
      if (abs(Speed) > Motor_B2->Max_PWM_Value)
        Speed = Motor_B2->Max_PWM_Value;
      else if (abs(Speed) < Motor_B2->Min_PWM_Value && abs(Speed) != 0)
        Speed = Motor_B2->Min_PWM_Value;
      variwheelerControl.setPWM(Motor_B2->PWM_Pin_Out, 1, abs(Speed));
      break;

    default:
      // TODO: En esta version todavia no se reportan errores
      break;
    }
    GeneralSpeed = A1_Speed + B1_Speed + A2_Speed + B2_Speed; // Sumatoria de vectores (Vector resultante)
    X_Vector = sqrt(2) * GeneralSpeed / 2;                    // Sumatoria de vectores descompuestos en X // TODO: En esta version todavia no se contempla una descomposicion vectorial fiable
    Y_Vector = sqrt(2) * GeneralSpeed / 2;                    // Sumatoria de vectores descompuestos en X // TODO: En esta version todavia no se contempla una descomposicion vectorial fiable
  }
}

/**
 * @brief Sets the speed of two selected motors and update it
 *
 * @param Mot1 MotorSelection. Indicates wich motor (A1, B1, A2, B2)
 * @param Speed1 int32_t ± (2^PWM_Resolution)-1. Speed to set in percetage or PWM res
 * @param Mot2 MotorSelection. Indicates wich motor (A1, B1, A2, B2)
 * @param Speed2 int32_t ± (2^PWM_Resolution)-1. Speed to set in percetage or PWM res
 *
 * @retval None
 */
void direction::SetIndividualMotorSpeed(MotorSelection Mot1, int32_t Speed1, MotorSelection Mot2, int32_t Speed2)
{
  SetIndividualMotorSpeed(Mot1, Speed1);
  SetIndividualMotorSpeed(Mot2, Speed2);
}

/**
 * @brief Sets the speed of three selected motors and update it
 *
 * @param Mot1 MotorSelection. Indicates wich motor (A1, B1, A2, B2)
 * @param Speed1 int32_t ± (2^PWM_Resolution)-1. Speed to set in percetage or PWM res
 * @param Mot2 MotorSelection. Indicates wich motor (A1, B1, A2, B2)
 * @param Speed2 int32_t ± (2^PWM_Resolution)-1. Speed to set in percetage or PWM res
 * @param Mot3 MotorSelection. Indicates wich motor (A1, B1, A2, B2)
 * @param Speed3 int32_t ± (2^PWM_Resolution)-1. Speed to set in percetage or PWM res
 *
 * @retval None
 */
void direction::SetIndividualMotorSpeed(MotorSelection Mot1, int32_t Speed1, MotorSelection Mot2, int32_t Speed2, MotorSelection Mot3, int32_t Speed3)
{
  SetIndividualMotorSpeed(Mot1, Speed1);
  SetIndividualMotorSpeed(Mot2, Speed2);
  SetIndividualMotorSpeed(Mot3, Speed3);
}

/**
 * @brief Sets the speed of three selected motors and update it
 *
 * @param Mot1 MotorSelection. Indicates wich motor (A1, B1, A2, B2)
 * @param Speed1 int32_t ± (2^PWM_Resolution)-1. Speed to set in percetage or PWM res
 * @param Mot2 MotorSelection. Indicates wich motor (A1, B1, A2, B2)
 * @param Speed2 int32_t ± (2^PWM_Resolution)-1. Speed to set in percetage or PWM res
 * @param Mot3 MotorSelection. Indicates wich motor (A1, B1, A2, B2)
 * @param Speed3 int32_t ± (2^PWM_Resolution)-1. Speed to set in percetage or PWM res
 * @param Mot4 MotorSelection. Indicates wich motor (A1, B1, A2, B2)
 * @param Speed4 int32_t ± (2^PWM_Resolution)-1. Speed to set in percetage or PWM res
 *
 * @retval None
 */
void direction::SetIndividualMotorSpeed(MotorSelection Mot1, int32_t Speed1, MotorSelection Mot2, int32_t Speed2, MotorSelection Mot3, int32_t Speed3, MotorSelection Mot4, int32_t Speed4)
{
  SetIndividualMotorSpeed(Mot1, Speed1);
  SetIndividualMotorSpeed(Mot2, Speed2);
  SetIndividualMotorSpeed(Mot3, Speed3);
  SetIndividualMotorSpeed(Mot4, Speed4);
}

/**
 * @brief Set a scalar like direction.
 *
 * @param Direction .Simple Direction between 8 posibilities (FORWARD, FORWARD_RIGHT, RIGHT, BACKWARD_RIGHT, BACKWARD, BACKWARD_LEFT, LEFT, FORWARD_LEFT)
 *
 * @retval None
 */
void direction::SetSimpleDirection(Simple Direction)
{
  switch (Direction)
  {
  case FORWARD:
    SetMoveVector(abs(GeneralSpeed), 0, 0);
    break;

  case FORWARD_RIGHT:
    SetMoveVector(abs(GeneralSpeed), abs(GeneralSpeed), 0);
    break;

  case RIGHT:
    SetMoveVector(0, abs(GeneralSpeed), 0);
    break;

  case BACKWARD_RIGHT:
    SetMoveVector(-abs(GeneralSpeed), abs(GeneralSpeed), 0);
    break;

  case BACKWARD:
    SetMoveVector(-abs(GeneralSpeed), 0, 0);
    break;

  case BACKWARD_LEFT:
    SetMoveVector(-abs(GeneralSpeed), -abs(GeneralSpeed), 0);
    break;

  case LEFT:
    SetMoveVector(0, -abs(GeneralSpeed), 0);
    break;

  case FORWARD_LEFT:
    SetMoveVector(abs(GeneralSpeed), -abs(GeneralSpeed), 0);
    break;

  default:
    // TODO: En esta version todavia no hace nada para reportar un error
    break;
  }
}

/**
 * @brief Set a scalar like orientation at specified speed.
 *
 * @param Direction Simple. Orientation between 7 posibilities (FORWARD_RIGHT, RIGHT, BACKWARD_RIGHT, BACKWARD, BACKWARD_LEFT, LEFT, FORWARD_LEFT)
 * @param RotationSpeed float ± (2^PWM_Resolution)-1. Speed in percentaje if UseSpeedPercentaje = true, PWM value if UseRelativeVelocity = true speed in rad/s if both false
 *
 * @retval None
 */
void direction::SetSimpleOrientation(Simple Orientation, float RotationSpeed)
{
  float angle;

  switch (Orientation)
  {
  case FORWARD_RIGHT:
    angle = 45;
    break;

  case RIGHT:
    angle = 90;
    break;

  case BACKWARD_RIGHT:
    angle = 135;
    break;

  case BACKWARD:
    if (random(2) == 1)
      angle = 180;
    else
      angle = -180;
    break;

  case BACKWARD_LEFT:
    angle = -135;
    break;

  case LEFT:
    angle = -90;
    break;

  case FORWARD_LEFT:
    angle = -45;
    break;

  default:
    // TODO: En esta version todavia no hace nada para reportar un error
    break;
  }
  if (!Use_Degrees)
  {
    angle = angle * M_PI / 180; // Degree to rads
  }
  Turn(angle, RotationSpeed);
}

/**
 * @brief Set a scalar like orientation at max speed.
 *
 * @param Direction Simple. Orientation between 7 posibilities (FORWARD_RIGHT, RIGHT, BACKWARD_RIGHT, BACKWARD, BACKWARD_LEFT, LEFT, FORWARD_LEFT)
 *
 * @retval None
 */
void direction::SetSimpleOrientation(Simple Orientation)
{
  if (Relative_Velocity && !Use_Speed_Percentaje)
    SetSimpleOrientation(Orientation, (pow(2, PWM_Resolution) - 1));
  else if (Use_Speed_Percentaje)
    SetSimpleOrientation(Orientation, 100);
  else if (!Use_Speed_Percentaje && !Relative_Velocity)
    SetSimpleOrientation(Orientation, (Angular_Max_Speed * Wheel_Radius * 2) / Horizontal_Separation);
}

/**
 * @brief Turn specified angle or radians at specified speed
 *
 * @param angle float. Angle to turn in degrees if UseDegrees = true, else radians
 * @param RotationSpeed float ± (2^PWM_Resolution)-1. Speed in percentaje if UseSpeedPercentaje = true, PWM value if UseRelativeVelocity = true speed in rad/s if both false
 *
 * @retval None
 */
void direction::Turn(float angle, float RotationSpeed)
{
  int32_t A1_PWM, B1_PWM, A2_PWM, B2_PWM; // Variables para escribir el PWM
  bool reverse = false;

  // Convertir la velocidad de rotacion a un formato valido:
  if (Use_Speed_Percentaje)
  {
    RotationSpeed = 4095 * RotationSpeed / 100;
  }
  else if (!Relative_Velocity)
  {
    RotationSpeed = (RotationSpeed * Horizontal_Separation / 2) / Wheel_Radius; // w*r=v (r en este caso es la separacion entre ruedas) (/2 por que son 2 ruedas por lado) ademas calcular la velocidad angular de cada rueda
    if (RotationSpeed > Angular_Max_Speed || RotationSpeed <= 0)
    {
      // TODO: En esta version todavia no hace nada para reportar un error
      return; // Cancela la ejecucion por seguridad
    }
    RotationSpeed = RotationSpeed * Reescaler_Volt * Volt_to_PWM; // Pasa a escala PWM
  }

  if (angle < 0)
  {
    reverse = true;
    angle = -angle;
  }
  if (Use_Degrees)
  {
    angle = angle * M_PI / 180; // Degree to rads
  }

  // Calcular el tiempo que debe tardar en girar dicho angulo dado que Squaere_Angle_Time_Out son 90º a maxima potencia
  float Squaere_Angle_Time_Out_No_Max = RotationSpeed * Squaere_Angle_Time_Out / 4095;
  uint32_t MaxRotationTime = angle / M_PI_2 * Squaere_Angle_Time_Out_No_Max;

  if (reverse)
  {
    A1_PWM = Compensator * RotationSpeed;    // -z (Calculo de la velocidad de la primera rueda)
    B1_PWM = (-Compensator * RotationSpeed); // +z (Calculo de la velocidad de la segunda rueda)
    A2_PWM = Compensator * RotationSpeed;    // -z (Calculo de la velocidad de la tercera rueda)
    B2_PWM = (-Compensator * RotationSpeed); // +z (Calculo de la velocidad de la cuarta rueda)
  }
  else
  {
    A1_PWM = (-Compensator * RotationSpeed); // -z (Calculo de la velocidad de la primera rueda)
    B1_PWM = Compensator * RotationSpeed;    // +z (Calculo de la velocidad de la segunda rueda)
    A2_PWM = (-Compensator * RotationSpeed); // -z (Calculo de la velocidad de la tercera rueda)
    B2_PWM = Compensator * RotationSpeed;    // +z (Calculo de la velocidad de la cuarta rueda)
  }

  // Escritura de la direccion
  if ((A1_PWM < 0 && Motor_A1->dir == CW) || (A1_PWM > 0 && Motor_A1->dir == CCW))
  {
    variwheelerControl.setPWM(Motor_A1->Dir2_Pin_Out, LOGIC_ZERO); // 1 Logico
    variwheelerControl.setPWM(Motor_A1->Dir1_Pin_Out, LOGIC_ONE);  // 0 logico
  }
  else if ((A1_PWM > 0 && Motor_A1->dir == CW) || (A1_PWM < 0 && Motor_A1->dir == CCW))
  {
    variwheelerControl.setPWM(Motor_A1->Dir1_Pin_Out, LOGIC_ZERO); // 0 logico
    variwheelerControl.setPWM(Motor_A1->Dir2_Pin_Out, LOGIC_ONE);  // 1 Logico
  }

  if ((B1_PWM < 0 && Motor_B1->dir == CW) || (B1_PWM > 0 && Motor_B1->dir == CCW))
  {
    variwheelerControl.setPWM(Motor_B1->Dir2_Pin_Out, LOGIC_ZERO); // 1 Logico
    variwheelerControl.setPWM(Motor_B1->Dir1_Pin_Out, LOGIC_ONE);  // 0 logico
  }
  else if ((B1_PWM > 0 && Motor_B1->dir == CW) || (B1_PWM < 0 && Motor_B1->dir == CCW))
  {
    variwheelerControl.setPWM(Motor_B1->Dir1_Pin_Out, LOGIC_ZERO); // 0 logico
    variwheelerControl.setPWM(Motor_B1->Dir2_Pin_Out, LOGIC_ONE);  // 1 Logico
  }

  if ((A2_PWM < 0 && Motor_A2->dir == CW) || (A2_PWM > 0 && Motor_A2->dir == CCW))
  {
    variwheelerControl.setPWM(Motor_A2->Dir2_Pin_Out, LOGIC_ZERO); // 1 Logico
    variwheelerControl.setPWM(Motor_A2->Dir1_Pin_Out, LOGIC_ONE);  // 0 logico
  }
  else if ((A2_PWM > 0 && Motor_A2->dir == CW) || (A2_PWM < 0 && Motor_A2->dir == CCW))
  {
    variwheelerControl.setPWM(Motor_A2->Dir1_Pin_Out, LOGIC_ZERO); // 0 logico
    variwheelerControl.setPWM(Motor_A2->Dir2_Pin_Out, LOGIC_ONE);  // 1 Logico
  }

  if ((B2_PWM < 0 && Motor_B2->dir == CW) || (B2_PWM > 0 && Motor_B2->dir == CCW))
  {
    variwheelerControl.setPWM(Motor_B2->Dir2_Pin_Out, LOGIC_ZERO); // 1 Logico
    variwheelerControl.setPWM(Motor_B2->Dir1_Pin_Out, LOGIC_ONE);  // 0 logico
  }
  else if ((B2_PWM > 0 && Motor_B2->dir == CW) || (B2_PWM < 0 && Motor_B2->dir == CCW))
  {
    variwheelerControl.setPWM(Motor_B2->Dir1_Pin_Out, LOGIC_ZERO); // 0 logico
    variwheelerControl.setPWM(Motor_B2->Dir2_Pin_Out, LOGIC_ONE);  // 1 Logico
  }

  // PWM

  if (abs(A1_PWM) > Motor_A1->Max_PWM_Value)
    A1_PWM = Motor_A1->Max_PWM_Value;
  else if (abs(A1_PWM) < Motor_A1->Min_PWM_Value && abs(A1_PWM) != 0)
    A1_PWM = Motor_A1->Min_PWM_Value;

  if (abs(B1_PWM) > Motor_B1->Max_PWM_Value)
    B1_PWM = Motor_B1->Max_PWM_Value;
  else if (abs(B1_PWM) < Motor_B1->Min_PWM_Value && abs(B1_PWM) != 0)
    B1_PWM = Motor_B1->Min_PWM_Value;

  if (abs(A2_PWM) > Motor_A2->Max_PWM_Value)
    A2_PWM = Motor_A2->Max_PWM_Value;
  else if (abs(A2_PWM) < Motor_A2->Min_PWM_Value && abs(A2_PWM) != 0)
    A2_PWM = Motor_A2->Min_PWM_Value;

  if (abs(B2_PWM) > Motor_B2->Max_PWM_Value)
    B2_PWM = Motor_B2->Max_PWM_Value;
  else if (abs(B2_PWM) < Motor_B2->Min_PWM_Value && abs(B2_PWM) != 0)
    B2_PWM = Motor_B2->Min_PWM_Value;

  variwheelerControl.setPWM(Motor_A1->PWM_Pin_Out, 1, abs(A1_PWM));
  variwheelerControl.setPWM(Motor_B1->PWM_Pin_Out, 1, abs(B1_PWM));
  variwheelerControl.setPWM(Motor_A2->PWM_Pin_Out, 1, abs(A2_PWM));
  variwheelerControl.setPWM(Motor_B2->PWM_Pin_Out, 1, abs(B2_PWM));

  delay(MaxRotationTime); // TODO: En esta version, el giro bloquea otras funciones

  variwheelerControl.setPWM(Motor_A1->PWM_Pin_Out, 0, 0);
  variwheelerControl.setPWM(Motor_B1->PWM_Pin_Out, 0, 0);
  variwheelerControl.setPWM(Motor_A2->PWM_Pin_Out, 0, 0);
  variwheelerControl.setPWM(Motor_B2->PWM_Pin_Out, 0, 0);
}

/**
 * @brief Turn specified angle or radians at max speed
 *
 * @param angle float. Angle to turn in degrees if UseDegrees = true, else radians
 *
 * @retval None
 */
void direction::Turn(float angle)
{
  if (Relative_Velocity && !Use_Speed_Percentaje)
    Turn(angle, (pow(2, PWM_Resolution) - 1));
  else if (Use_Speed_Percentaje)
    Turn(angle, 100);
  else if (!Use_Speed_Percentaje && !Relative_Velocity)
    Turn(angle, (Angular_Max_Speed * Wheel_Radius * 2) / Horizontal_Separation);
}

/**
 * @brief Turn unespecified angle or radians at specified speed
 *
 * @param RotationSpeed float ± (2^PWM_Resolution)-1. Speed in percentaje if UseSpeedPercentaje = true, PWM value if UseRelativeVelocity = true speed in rad/s if both false
 *
 * @retval None
 */
void direction::UndefinedTurn(float RotationSpeed)
{
  int32_t A1_PWM, B1_PWM, A2_PWM, B2_PWM; // Variables para escribir el PWM

  // Convertir la velocidad de rotacion a un formato valido:
  if (Use_Speed_Percentaje)
  {
    RotationSpeed = 4095 * RotationSpeed / 100;
  }
  else if (!Relative_Velocity)
  {
    RotationSpeed = (RotationSpeed * Horizontal_Separation / 2) / Wheel_Radius; // w*r=v (r en este caso es la separacion entre ruedas) (/2 por que son 2 ruedas por lado) ademas calcular la velocidad angular de cada rueda
    if (RotationSpeed > Angular_Max_Speed || RotationSpeed <= 0)
    {
      // TODO: En esta version todavia no hace nada para reportar un error
      return; // Cancela la ejecucion por seguridad
    }
    RotationSpeed = RotationSpeed * Reescaler_Volt * Volt_to_PWM; // Pasa a escala PWM
  }

  A1_PWM = (-Compensator * RotationSpeed); // -z (Calculo de la velocidad de la primera rueda)
  B1_PWM = Compensator * RotationSpeed;    // +z (Calculo de la velocidad de la segunda rueda)
  A2_PWM = (-Compensator * RotationSpeed); // -z (Calculo de la velocidad de la tercera rueda)
  B2_PWM = Compensator * RotationSpeed;    // +z (Calculo de la velocidad de la cuarta rueda)

  // Escritura de la direccion
  if ((A1_PWM < 0 && Motor_A1->dir == CW) || (A1_PWM > 0 && Motor_A1->dir == CCW))
  {
    variwheelerControl.setPWM(Motor_A1->Dir2_Pin_Out, LOGIC_ZERO); // 1 Logico
    variwheelerControl.setPWM(Motor_A1->Dir1_Pin_Out, LOGIC_ONE);  // 0 logico
  }
  else if ((A1_PWM > 0 && Motor_A1->dir == CW) || (A1_PWM < 0 && Motor_A1->dir == CCW))
  {
    variwheelerControl.setPWM(Motor_A1->Dir1_Pin_Out, LOGIC_ZERO); // 0 logico
    variwheelerControl.setPWM(Motor_A1->Dir2_Pin_Out, LOGIC_ONE);  // 1 Logico
  }

  if ((B1_PWM < 0 && Motor_B1->dir == CW) || (B1_PWM > 0 && Motor_B1->dir == CCW))
  {
    variwheelerControl.setPWM(Motor_B1->Dir2_Pin_Out, LOGIC_ZERO); // 1 Logico
    variwheelerControl.setPWM(Motor_B1->Dir1_Pin_Out, LOGIC_ONE);  // 0 logico
  }
  else if ((B1_PWM > 0 && Motor_B1->dir == CW) || (B1_PWM < 0 && Motor_B1->dir == CCW))
  {
    variwheelerControl.setPWM(Motor_B1->Dir1_Pin_Out, LOGIC_ZERO); // 0 logico
    variwheelerControl.setPWM(Motor_B1->Dir2_Pin_Out, LOGIC_ONE);  // 1 Logico
  }

  if ((A2_PWM < 0 && Motor_A2->dir == CW) || (A2_PWM > 0 && Motor_A2->dir == CCW))
  {
    variwheelerControl.setPWM(Motor_A2->Dir2_Pin_Out, LOGIC_ZERO); // 1 Logico
    variwheelerControl.setPWM(Motor_A2->Dir1_Pin_Out, LOGIC_ONE);  // 0 logico
  }
  else if ((A2_PWM > 0 && Motor_A2->dir == CW) || (A2_PWM < 0 && Motor_A2->dir == CCW))
  {
    variwheelerControl.setPWM(Motor_A2->Dir1_Pin_Out, LOGIC_ZERO); // 0 logico
    variwheelerControl.setPWM(Motor_A2->Dir2_Pin_Out, LOGIC_ONE);  // 1 Logico
  }

  if ((B2_PWM < 0 && Motor_B2->dir == CW) || (B2_PWM > 0 && Motor_B2->dir == CCW))
  {
    variwheelerControl.setPWM(Motor_B2->Dir2_Pin_Out, LOGIC_ZERO); // 1 Logico
    variwheelerControl.setPWM(Motor_B2->Dir1_Pin_Out, LOGIC_ONE);  // 0 logico
  }
  else if ((B2_PWM > 0 && Motor_B2->dir == CW) || (B2_PWM < 0 && Motor_B2->dir == CCW))
  {
    variwheelerControl.setPWM(Motor_B2->Dir1_Pin_Out, LOGIC_ZERO); // 0 logico
    variwheelerControl.setPWM(Motor_B2->Dir2_Pin_Out, LOGIC_ONE);  // 1 Logico
  }

  // PWM

  if (abs(A1_PWM) > Motor_A1->Max_PWM_Value)
    A1_PWM = Motor_A1->Max_PWM_Value;
  else if (abs(A1_PWM) < Motor_A1->Min_PWM_Value && abs(A1_PWM) != 0)
    A1_PWM = Motor_A1->Min_PWM_Value;

  if (abs(B1_PWM) > Motor_B1->Max_PWM_Value)
    B1_PWM = Motor_B1->Max_PWM_Value;
  else if (abs(B1_PWM) < Motor_B1->Min_PWM_Value && abs(B1_PWM) != 0)
    B1_PWM = Motor_B1->Min_PWM_Value;

  if (abs(A2_PWM) > Motor_A2->Max_PWM_Value)
    A2_PWM = Motor_A2->Max_PWM_Value;
  else if (abs(A2_PWM) < Motor_A2->Min_PWM_Value && abs(A2_PWM) != 0)
    A2_PWM = Motor_A2->Min_PWM_Value;

  if (abs(B2_PWM) > Motor_B2->Max_PWM_Value)
    B2_PWM = Motor_B2->Max_PWM_Value;
  else if (abs(B2_PWM) < Motor_B2->Min_PWM_Value && abs(B2_PWM) != 0)
    B2_PWM = Motor_B2->Min_PWM_Value;

  variwheelerControl.setPWM(Motor_A1->PWM_Pin_Out, 1, abs(A1_PWM));
  variwheelerControl.setPWM(Motor_B1->PWM_Pin_Out, 1, abs(B1_PWM));
  variwheelerControl.setPWM(Motor_A2->PWM_Pin_Out, 1, abs(A2_PWM));
  variwheelerControl.setPWM(Motor_B2->PWM_Pin_Out, 1, abs(B2_PWM));
}

/**
 * @brief Turn unespecified angle or radians at max speed
 *
 *
 * @retval None
 */
void direction::UndefinedTurn()
{
  if (Relative_Velocity && !Use_Speed_Percentaje)
    UndefinedTurn((pow(2, PWM_Resolution) - 1));
  else if (Use_Speed_Percentaje)
    UndefinedTurn(100);
  else if (!Use_Speed_Percentaje && !Relative_Velocity)
    UndefinedTurn((Angular_Max_Speed * Wheel_Radius * 2) / Horizontal_Separation);
}

/**
 * @brief Gives the vector of movement and act over the motors
 *
 * @param X int32_t ± (2^PWM_Resolution)-1. Left/right movement (0 - PWM res) if relative (0 - Max linear velocity in mm/s) if real
 * @param Y int32_t ± (2^PWM_Resolution)-1. Front/back movement (0 - PWM res) if relative (0 - Max linear velocity in mm/s) if real
 * @param Z int32_t ± (2^PWM_Resolution)-1. CW/CCW rotation (0 - PWM res) if relative (0 - Max linear velocity in mm/s) if real
 *
 * @retval None
 */
void direction::SetMoveVector(int32_t X, int32_t Y, int32_t Z)
{
  int32_t A1_PWM, B1_PWM, A2_PWM, B2_PWM;                                       // Variables para escribir el PWM
  uint8_t Zeros;                                                                // Conteo de ruedas quietas
  float A1_Angular_Speed, B1_Angular_Speed, A2_Angular_Speed, B2_Angular_Speed; // Variables para comprobar la velocidad angular

  if (abs(X) <= pow(2, PWM_Resolution) - 1 && abs(Y) <= pow(2, PWM_Resolution) - 1 && abs(Z) <= pow(2, PWM_Resolution) - 1)
  {
    if ((X >= 100 || Y >= 100 || Y >= 100 || X <= -100 || Y <= -100 || Y <= -100) && Use_Speed_Percentaje)
    {
      // TODO: En esta version todavia no se reportan errores
      return;
    }
    // Se almacenan las variables en las globales
    X_Vector = X;
    Y_Vector = Y;
    Z_Vector = Z;

    if (Use_Speed_Percentaje) // Tansformacion de % a PWM relativo
    {
      X_Vector = (pow(2, PWM_Resolution) - 1) * X_Vector / 100;
      Y_Vector = (pow(2, PWM_Resolution) - 1) * Y_Vector / 100;
      Z_Vector = (pow(2, PWM_Resolution) - 1) * Z_Vector / 100;
    }

    if (Relative_Velocity || Use_Speed_Percentaje) // Si la velocidad es trabajo en relativa
    {
      // Calculo de los vectores de las ruedas
      A1_Vector = ((1 - Compensator) * sqrt(2) * (Y_Vector + X_Vector) / 2 - Compensator * Z_Vector); // y+x-z (Calculo de la velocidad de la primera rueda)
      B1_Vector = ((1 - Compensator) * sqrt(2) * (Y_Vector - X_Vector) / 2 + Compensator * Z_Vector); // y-x+z (Calculo de la velocidad de la segunda rueda)
      A2_Vector = ((1 - Compensator) * sqrt(2) * (Y_Vector - X_Vector) / 2 - Compensator * Z_Vector); // y-x-z (Calculo de la velocidad de la tercera rueda)
      B2_Vector = ((1 - Compensator) * sqrt(2) * (Y_Vector + X_Vector) / 2 + Compensator * Z_Vector); // y+x+z (Calculo de la velocidad de la cuarta rueda)
      GeneralSpeed = A1_Vector + B1_Vector + A2_Vector + B2_Vector;                                   // Sumatoria de vectores (Vector resultante)

      A1_Speed = Reescaler_PWM * Reescaler_math * A1_Vector;
      B1_Speed = Reescaler_PWM * Reescaler_math * B1_Vector;
      A2_Speed = Reescaler_PWM * Reescaler_math * A2_Vector;
      B2_Speed = Reescaler_PWM * Reescaler_math * B2_Vector;
      GeneralSpeed = Reescaler_PWM * Reescaler_math * GeneralSpeed; // Velocidad reescalada

      A1_PWM = abs(A1_Speed);
      B1_PWM = abs(B1_Speed);
      A2_PWM = abs(A2_Speed);
      B2_PWM = abs(B2_Speed);

      if (Use_Speed_Percentaje) // Devuelve el vector a %
      {
        X_Vector = X;
        Y_Vector = Y;
        Z_Vector = Z;
        GeneralSpeed = GeneralSpeed * 100 / (pow(2, PWM_Resolution) - 1);
      }
    }
    else
    {
      // Calcular la velocidad
      A1_Speed = (1 - Compensator) * sqrt(2) * (Y_Vector + X_Vector) / 2 - Compensator * Z_Vector;
      B1_Speed = (1 - Compensator) * sqrt(2) * (Y_Vector - X_Vector) / 2 + Compensator * Z_Vector;
      A2_Speed = (1 - Compensator) * sqrt(2) * (Y_Vector - X_Vector) / 2 - Compensator * Z_Vector;
      B2_Speed = (1 - Compensator) * sqrt(2) * (Y_Vector + X_Vector) / 2 + Compensator * Z_Vector;
      A1_Vector = A1_Speed;
      B1_Vector = B1_Speed;
      A2_Vector = A2_Speed;
      B2_Vector = B2_Speed;

      GeneralSpeed = A1_Speed + B1_Speed + A2_Speed + B2_Speed; // Sumatoria de vectores (Vector resultante)
      // Pasar velocidad lineal a angular
      A1_Angular_Speed = abs(A1_Speed / Wheel_Radius);
      B1_Angular_Speed = abs(B1_Speed / Wheel_Radius);
      A2_Angular_Speed = abs(A2_Speed / Wheel_Radius);
      B2_Angular_Speed = abs(B2_Speed / Wheel_Radius);
      // Comprobar que no rebasa el limite de velocidad angular
      if (A1_Angular_Speed > Angular_Max_Speed || B1_Angular_Speed > Angular_Max_Speed || A2_Angular_Speed > Angular_Max_Speed || B2_Angular_Speed > Angular_Max_Speed)
      {
        // TODO: En esta version todavia no hace nada para reportar un error
      }
      else
      {
        // Comprobacion de motores parados
        /*if (A1_Angular_Speed == 0)
          Zeros++;
        if (B1_Angular_Speed == 0)
          Zeros++;
        if (A2_Angular_Speed == 0)
          Zeros++;
        if (B2_Angular_Speed == 0)
          Zeros++;*/

        // Paso a PWM y division de potencia en funcion de los motores parados
        A1_PWM = A1_Angular_Speed * Reescaler_Volt * Volt_to_PWM; // / (4 - Zeros);
        B1_PWM = B1_Angular_Speed * Reescaler_Volt * Volt_to_PWM; // / (4 - Zeros);
        A2_PWM = A2_Angular_Speed * Reescaler_Volt * Volt_to_PWM; // / (4 - Zeros);
        B2_PWM = B2_Angular_Speed * Reescaler_Volt * Volt_to_PWM; // / (4 - Zeros);
      }
    }

    // Escribir el PWM contemplando la direccion
    if ((A1_Speed < 0 && Motor_A1->dir == CW) || (A1_Speed > 0 && Motor_A1->dir == CCW))
    {
      variwheelerControl.setPWM(Motor_A1->Dir2_Pin_Out, LOGIC_ZERO); // 1 Logico
      variwheelerControl.setPWM(Motor_A1->Dir1_Pin_Out, LOGIC_ONE);  // 0 logico
    }
    else if ((A1_Speed > 0 && Motor_A1->dir == CW) || (A1_Speed < 0 && Motor_A1->dir == CCW))
    {
      variwheelerControl.setPWM(Motor_A1->Dir1_Pin_Out, LOGIC_ZERO); // 0 logico
      variwheelerControl.setPWM(Motor_A1->Dir2_Pin_Out, LOGIC_ONE);  // 1 Logico
    }

    if ((B1_Speed < 0 && Motor_B1->dir == CW) || (B1_Speed > 0 && Motor_B1->dir == CCW))
    {
      variwheelerControl.setPWM(Motor_B1->Dir2_Pin_Out, LOGIC_ZERO); // 1 Logico
      variwheelerControl.setPWM(Motor_B1->Dir1_Pin_Out, LOGIC_ONE);  // 0 logico
    }
    else if ((B1_Speed > 0 && Motor_B1->dir == CW) || (B1_Speed < 0 && Motor_B1->dir == CCW))
    {
      variwheelerControl.setPWM(Motor_B1->Dir1_Pin_Out, LOGIC_ZERO); // 0 logico
      variwheelerControl.setPWM(Motor_B1->Dir2_Pin_Out, LOGIC_ONE);  // 1 Logico
    }

    if ((A2_Speed < 0 && Motor_A2->dir == CW) || (A2_Speed > 0 && Motor_A2->dir == CCW))
    {
      variwheelerControl.setPWM(Motor_A2->Dir2_Pin_Out, LOGIC_ZERO); // 1 Logico
      variwheelerControl.setPWM(Motor_A2->Dir1_Pin_Out, LOGIC_ONE);  // 0 logico
    }
    else if ((A2_Speed > 0 && Motor_A2->dir == CW) || (A2_Speed < 0 && Motor_A2->dir == CCW))
    {
      variwheelerControl.setPWM(Motor_A2->Dir1_Pin_Out, LOGIC_ZERO); // 0 logico
      variwheelerControl.setPWM(Motor_A2->Dir2_Pin_Out, LOGIC_ONE);  // 1 Logico
    }

    if ((B2_Speed < 0 && Motor_B2->dir == CW) || (B2_Speed > 0 && Motor_B2->dir == CCW))
    {
      variwheelerControl.setPWM(Motor_B2->Dir2_Pin_Out, LOGIC_ZERO); // 1 Logico
      variwheelerControl.setPWM(Motor_B2->Dir1_Pin_Out, LOGIC_ONE);  // 0 logico
    }
    else if ((B2_Speed > 0 && Motor_B2->dir == CW) || (B2_Speed < 0 && Motor_B2->dir == CCW))
    {
      variwheelerControl.setPWM(Motor_B2->Dir1_Pin_Out, LOGIC_ZERO); // 0 logico
      variwheelerControl.setPWM(Motor_B2->Dir2_Pin_Out, LOGIC_ONE);  // 1 Logico
    }

    // PWM

    if (abs(A1_PWM) > Motor_A1->Max_PWM_Value)
      A1_PWM = Motor_A1->Max_PWM_Value;
    else if (abs(A1_PWM) < Motor_A1->Min_PWM_Value && abs(A1_PWM) != 0)
      A1_PWM = Motor_A1->Min_PWM_Value;

    if (abs(B1_PWM) > Motor_B1->Max_PWM_Value)
      B1_PWM = Motor_B1->Max_PWM_Value;
    else if (abs(B1_PWM) < Motor_B1->Min_PWM_Value && abs(B1_PWM) != 0)
      B1_PWM = Motor_B1->Min_PWM_Value;

    if (abs(A2_PWM) > Motor_A2->Max_PWM_Value)
      A2_PWM = Motor_A2->Max_PWM_Value;
    else if (abs(A2_PWM) < Motor_A2->Min_PWM_Value && abs(A2_PWM) != 0)
      A2_PWM = Motor_A2->Min_PWM_Value;

    if (abs(B2_PWM) > Motor_B2->Max_PWM_Value)
      B2_PWM = Motor_B2->Max_PWM_Value;
    else if (abs(B2_PWM) < Motor_B2->Min_PWM_Value && abs(B2_PWM) != 0)
      B2_PWM = Motor_B2->Min_PWM_Value;

    variwheelerControl.setPWM(Motor_A1->PWM_Pin_Out, 1, abs(A1_PWM));
    variwheelerControl.setPWM(Motor_B1->PWM_Pin_Out, 1, abs(B1_PWM));
    variwheelerControl.setPWM(Motor_A2->PWM_Pin_Out, 1, abs(A2_PWM));
    variwheelerControl.setPWM(Motor_B2->PWM_Pin_Out, 1, abs(B2_PWM));
  }
  else
  {
    // TODO: En esta version todavia no hace nada para reportar un error
  }
}

/**
 * @brief Stops the motors by releasing the motors
 *
 *
 * @retval None
 */
void direction::Brake()
{
  variwheelerControl.setPWM(Motor_A1->Dir2_Pin_Out, LOGIC_ZERO); // 0 Logico
  variwheelerControl.setPWM(Motor_A1->Dir1_Pin_Out, LOGIC_ZERO); // 0 logico

  variwheelerControl.setPWM(Motor_B1->Dir2_Pin_Out, LOGIC_ZERO); // 0 Logico
  variwheelerControl.setPWM(Motor_B1->Dir1_Pin_Out, LOGIC_ZERO); // 0 logico

  variwheelerControl.setPWM(Motor_A2->Dir2_Pin_Out, LOGIC_ZERO); // 0 Logico
  variwheelerControl.setPWM(Motor_A2->Dir1_Pin_Out, LOGIC_ZERO); // 0 logico

  variwheelerControl.setPWM(Motor_B2->Dir2_Pin_Out, LOGIC_ZERO); // 0 Logico
  variwheelerControl.setPWM(Motor_B2->Dir1_Pin_Out, LOGIC_ZERO); // 0 logico

  A1_Speed = 0;
  B1_Speed = 0;
  A2_Speed = 0;
  B2_Speed = 0;
}

/**
 * @brief Stops the motors by blocking the motors
 *
 *
 * @retval None
 */
void direction::HardBrake()
{
  variwheelerControl.setPWM(Motor_A1->Dir2_Pin_Out, LOGIC_ONE); // 1 Logico
  variwheelerControl.setPWM(Motor_A1->Dir1_Pin_Out, LOGIC_ONE); // 1 logico

  variwheelerControl.setPWM(Motor_B1->Dir2_Pin_Out, LOGIC_ONE); // 1 Logico
  variwheelerControl.setPWM(Motor_B1->Dir1_Pin_Out, LOGIC_ONE); // 1 logico

  variwheelerControl.setPWM(Motor_A2->Dir2_Pin_Out, LOGIC_ONE); // 1 Logico
  variwheelerControl.setPWM(Motor_A2->Dir1_Pin_Out, LOGIC_ONE); // 1 logico

  variwheelerControl.setPWM(Motor_B2->Dir2_Pin_Out, LOGIC_ONE); // 1 Logico
  variwheelerControl.setPWM(Motor_B2->Dir1_Pin_Out, LOGIC_ONE); // 1 logico

  A1_Speed = 0;
  B1_Speed = 0;
  A2_Speed = 0;
  B2_Speed = 0;
}

/**
 * @brief Stops the motors by inverting the motors and then releasing
 *
 *
 * @retval None
 */
void direction::ReverseBrake()
{

  // Invertir la direccion
  if ((A1_Speed < 0 && Motor_A1->dir == CW) || (A1_Speed > 0 && Motor_A1->dir == CCW))
  {
    variwheelerControl.setPWM(Motor_A1->Dir2_Pin_Out, LOGIC_ZERO); // 1 Logico
    variwheelerControl.setPWM(Motor_A1->Dir1_Pin_Out, LOGIC_ONE);  // 0 logico
  }
  else if ((A1_Speed > 0 && Motor_A1->dir == CW) || (A1_Speed < 0 && Motor_A1->dir == CCW))
  {
    variwheelerControl.setPWM(Motor_A1->Dir1_Pin_Out, LOGIC_ZERO); // 0 logico
    variwheelerControl.setPWM(Motor_A1->Dir2_Pin_Out, LOGIC_ONE);  // 1 Logico
  }

  if ((B1_Speed < 0 && Motor_B1->dir == CW) || (B1_Speed > 0 && Motor_B1->dir == CCW))
  {
    variwheelerControl.setPWM(Motor_B1->Dir2_Pin_Out, LOGIC_ZERO); // 1 Logico
    variwheelerControl.setPWM(Motor_B1->Dir1_Pin_Out, LOGIC_ONE);  // 0 logico
  }
  else if ((B1_Speed > 0 && Motor_B1->dir == CW) || (B1_Speed < 0 && Motor_B1->dir == CCW))
  {
    variwheelerControl.setPWM(Motor_B1->Dir1_Pin_Out, LOGIC_ZERO); // 0 logico
    variwheelerControl.setPWM(Motor_B1->Dir2_Pin_Out, LOGIC_ONE);  // 1 Logico
  }

  if ((A2_Speed < 0 && Motor_A2->dir == CW) || (A2_Speed > 0 && Motor_A2->dir == CCW))
  {
    variwheelerControl.setPWM(Motor_A2->Dir2_Pin_Out, LOGIC_ZERO); // 1 Logico
    variwheelerControl.setPWM(Motor_A2->Dir1_Pin_Out, LOGIC_ONE);  // 0 logico
  }
  else if ((A2_Speed > 0 && Motor_A2->dir == CW) || (A2_Speed < 0 && Motor_A2->dir == CCW))
  {
    variwheelerControl.setPWM(Motor_A2->Dir1_Pin_Out, LOGIC_ZERO); // 0 logico
    variwheelerControl.setPWM(Motor_A2->Dir2_Pin_Out, LOGIC_ONE);  // 1 Logico
  }

  if ((B2_Speed < 0 && Motor_B2->dir == CW) || (B2_Speed > 0 && Motor_B2->dir == CCW))
  {
    variwheelerControl.setPWM(Motor_B2->Dir2_Pin_Out, LOGIC_ZERO); // 1 Logico
    variwheelerControl.setPWM(Motor_B2->Dir1_Pin_Out, LOGIC_ONE);  // 0 logico
  }
  else if ((B2_Speed > 0 && Motor_B2->dir == CW) || (B2_Speed < 0 && Motor_B2->dir == CCW))
  {
    variwheelerControl.setPWM(Motor_B2->Dir1_Pin_Out, LOGIC_ZERO); // 0 logico
    variwheelerControl.setPWM(Motor_B2->Dir2_Pin_Out, LOGIC_ONE);  // 1 Logico
  }

  delay(Reverse_Brake_Time);

  variwheelerControl.setPWM(Motor_A1->Dir2_Pin_Out, LOGIC_ZERO); // 0 Logico
  variwheelerControl.setPWM(Motor_A1->Dir1_Pin_Out, LOGIC_ZERO); // 0 logico

  variwheelerControl.setPWM(Motor_B1->Dir2_Pin_Out, LOGIC_ZERO); // 0 Logico
  variwheelerControl.setPWM(Motor_B1->Dir1_Pin_Out, LOGIC_ZERO); // 0 logico

  variwheelerControl.setPWM(Motor_A2->Dir2_Pin_Out, LOGIC_ZERO); // 0 Logico
  variwheelerControl.setPWM(Motor_A2->Dir1_Pin_Out, LOGIC_ZERO); // 0 logico

  variwheelerControl.setPWM(Motor_B2->Dir2_Pin_Out, LOGIC_ZERO); // 0 Logico
  variwheelerControl.setPWM(Motor_B2->Dir1_Pin_Out, LOGIC_ZERO); // 0 logico

  A1_Speed = 0;
  B1_Speed = 0;
  A2_Speed = 0;
  B2_Speed = 0;
}

/**
 * @brief Stops the motors by inverting the motors and then blocking
 *
 *
 * @retval None
 */
void direction::ReverseHardBrake()
{

  // Invertir la direccion
  if ((A1_Speed < 0 && Motor_A1->dir == CW) || (A1_Speed > 0 && Motor_A1->dir == CCW))
  {
    variwheelerControl.setPWM(Motor_A1->Dir2_Pin_Out, LOGIC_ZERO); // 1 Logico
    variwheelerControl.setPWM(Motor_A1->Dir1_Pin_Out, LOGIC_ONE);  // 0 logico
  }
  else if ((A1_Speed > 0 && Motor_A1->dir == CW) || (A1_Speed < 0 && Motor_A1->dir == CCW))
  {
    variwheelerControl.setPWM(Motor_A1->Dir1_Pin_Out, LOGIC_ZERO); // 0 logico
    variwheelerControl.setPWM(Motor_A1->Dir2_Pin_Out, LOGIC_ONE);  // 1 Logico
  }

  if ((B1_Speed < 0 && Motor_B1->dir == CW) || (B1_Speed > 0 && Motor_B1->dir == CCW))
  {
    variwheelerControl.setPWM(Motor_B1->Dir2_Pin_Out, LOGIC_ZERO); // 1 Logico
    variwheelerControl.setPWM(Motor_B1->Dir1_Pin_Out, LOGIC_ONE);  // 0 logico
  }
  else if ((B1_Speed > 0 && Motor_B1->dir == CW) || (B1_Speed < 0 && Motor_B1->dir == CCW))
  {
    variwheelerControl.setPWM(Motor_B1->Dir1_Pin_Out, LOGIC_ZERO); // 0 logico
    variwheelerControl.setPWM(Motor_B1->Dir2_Pin_Out, LOGIC_ONE);  // 1 Logico
  }

  if ((A2_Speed < 0 && Motor_A2->dir == CW) || (A2_Speed > 0 && Motor_A2->dir == CCW))
  {
    variwheelerControl.setPWM(Motor_A2->Dir2_Pin_Out, LOGIC_ZERO); // 1 Logico
    variwheelerControl.setPWM(Motor_A2->Dir1_Pin_Out, LOGIC_ONE);  // 0 logico
  }
  else if ((A2_Speed > 0 && Motor_A2->dir == CW) || (A2_Speed < 0 && Motor_A2->dir == CCW))
  {
    variwheelerControl.setPWM(Motor_A2->Dir1_Pin_Out, LOGIC_ZERO); // 0 logico
    variwheelerControl.setPWM(Motor_A2->Dir2_Pin_Out, LOGIC_ONE);  // 1 Logico
  }

  if ((B2_Speed < 0 && Motor_B2->dir == CW) || (B2_Speed > 0 && Motor_B2->dir == CCW))
  {
    variwheelerControl.setPWM(Motor_B2->Dir2_Pin_Out, LOGIC_ZERO); // 1 Logico
    variwheelerControl.setPWM(Motor_B2->Dir1_Pin_Out, LOGIC_ONE);  // 0 logico
  }
  else if ((B2_Speed > 0 && Motor_B2->dir == CW) || (B2_Speed < 0 && Motor_B2->dir == CCW))
  {
    variwheelerControl.setPWM(Motor_B2->Dir1_Pin_Out, LOGIC_ZERO); // 0 logico
    variwheelerControl.setPWM(Motor_B2->Dir2_Pin_Out, LOGIC_ONE);  // 1 Logico
  }

  delay(Reverse_Brake_Time);

  variwheelerControl.setPWM(Motor_A1->Dir2_Pin_Out, LOGIC_ONE); // 1 Logico
  variwheelerControl.setPWM(Motor_A1->Dir1_Pin_Out, LOGIC_ONE); // 1 logico

  variwheelerControl.setPWM(Motor_B1->Dir2_Pin_Out, LOGIC_ONE); // 1 Logico
  variwheelerControl.setPWM(Motor_B1->Dir1_Pin_Out, LOGIC_ONE); // 1 logico

  variwheelerControl.setPWM(Motor_A2->Dir2_Pin_Out, LOGIC_ONE); // 1 Logico
  variwheelerControl.setPWM(Motor_A2->Dir1_Pin_Out, LOGIC_ONE); // 1 logico

  variwheelerControl.setPWM(Motor_B2->Dir2_Pin_Out, LOGIC_ONE); // 1 Logico
  variwheelerControl.setPWM(Motor_B2->Dir1_Pin_Out, LOGIC_ONE); // 1 logico

  A1_Speed = 0;
  B1_Speed = 0;
  A2_Speed = 0;
  B2_Speed = 0;
}

/**
 * @brief Used to get the actual motor speed
 *
 * @param Mot MotorSelection. Motor to check (A1, B1, A2, B2)
 *
 * @return Selected motor Speed
 * @retval int32_t ± (2^PWM_Resolution)-1 value
 */
int32_t direction::GetMotorSpeed(MotorSelection Mot)
{
  switch (Mot)
  {
  case A1_Motor:
    return A1_Speed;
    break;

  case B1_Motor:
    return B1_Speed;
    break;

  case A2_Motor:
    return A2_Speed;
    break;

  case B2_Motor:
    return B2_Speed;
    break;

  default:
    // TODO: En esta version todavia no se reportan errores
    return 0;
    break;
  }
}

/**
 * @brief Aproximes the direcction to a simple direction existent
 *
 * @return Aproximated actual direction
 * @retval Simple value (FORWARD, FORWARD_RIGHT, RIGHT, BACKWARD_RIGHT, BACKWARD, BACKWARD_LEFT, LEFT, FORWARD_LEFT)
 */
Simple direction::GetAproximateDirection()
{
  return FORWARD; // TODO: En esta version todavia no esta habilitada la estimacion de direccion simple
}

/**
 * @brief Gives the vector of movement and act over the motors
 *
 * @param X int32_t ± (2^PWM_Resolution)-1, left/right movement pointer (0 - PWM res) if relative (0 - Max linear velocity in mm/s) if real
 * @param Y int32_t ± (2^PWM_Resolution)-1, Front/back movement pointer (0 - PWM res) if relative (0 - Max linear velocity in mm/s) if real
 *
 * @retval None
 */
void direction::GetMoveVector(int32_t *X, int32_t *Y)
{
  *X = X_Vector;
  *Y = Y_Vector;
}

Adafruit_PWMServoDriver *direction::ReturnAdafruit_PWMServoDriver()
{
  return &variwheelerControl;
}

/**
 * @brief Constructor of the class, give to the class the necessary params to work, motors, and work mode
 *
 * @param Servo1 Servo struct pointer. Pointer to the first motor struct, front right.
 * @param Servo2 Servo struct pointer. Pointer to the second motor struct, front left.
 * @param Servo3 Servo struct pointer. Pointer to the third motor struct, back right.
 * @param Servo4 Servo struct pointer. Pointer to the fourth motor struct, back left.
 * @param DirClass direction class pointer. Pointer to the fourth motor struct, back left.
 *
 * @pre Is neccesary to have defined the structures of the fourth servos and the class to the direction previusly
 */
geometry::geometry(Servo *Servo1, Servo *Servo2, Servo *Servo3, Servo *Servo4, direction *DirClass)
{
  if (Servo1 == nullptr)
  {
    // TODO: Todavia no hay manejo de errores en esta version
  }
  else
    Servo_1 = Servo1;

  if (Servo2 == nullptr)
  {
    // TODO: Todavia no hay manejo de errores en esta version
  }
  else
    Servo_2 = Servo2;

  if (Servo3 == nullptr)
  {
    // TODO: Todavia no hay manejo de errores en esta version
  }
  else
    Servo_3 = Servo3;

  if (Servo4 == nullptr)
  {
    // TODO: Todavia no hay manejo de errores en esta version
  }
  else
    Servo_4 = Servo4;

  if (DirClass == nullptr)
  {
    // TODO: Todavia no hay manejo de errores en esta version
  }
  else
    Compresion = DirClass;

  if (abs(Servo_1->Up_End_Angle - Servo_1->Down_End_Angle) > abs(Servo_2->Up_End_Angle - Servo_2->Down_End_Angle))
    Restringed_Front_travel = abs(Servo_1->Up_End_Angle - Servo_1->Down_End_Angle);
  else
    Restringed_Front_travel = abs(Servo_2->Up_End_Angle - Servo_2->Down_End_Angle);
  if (abs(Servo_3->Up_End_Angle - Servo_3->Down_End_Angle) > abs(Servo_4->Up_End_Angle - Servo_4->Down_End_Angle))
    Restringed_Back_travel = abs(Servo_3->Up_End_Angle - Servo_3->Down_End_Angle);
  else
    Restringed_Back_travel = abs(Servo_4->Up_End_Angle - Servo_4->Down_End_Angle);

  Servo_1_RealAngle = Servo_1->Down_End_Angle;
  Servo_2_RealAngle = Servo_2->Down_End_Angle;
  Servo_3_RealAngle = Servo_3->Down_End_Angle;
  Servo_4_RealAngle = Servo_4->Down_End_Angle;

  variwheelerControl = Compresion->ReturnAdafruit_PWMServoDriver();
}

/**
 * @brief Configurator of the class, non mandatory.
 *
 * @param UseAnglePercentaje Boolean. Defines if the speed will work in percentage format or PWM (ture indicates percentaje) (false by default)
 * @param DownIsReference Boolean. Defines if the variwheeler will use real velocity or relative (relative means no control over specific velocity) (true indicates relative) (true by default)
 *
 * @retval None
 */
void geometry::Config(bool UseAnglePercentaje, bool DownIsReference)
{
  Use_Angle_Percentaje = UseAnglePercentaje;
  Down_Is_Reference = DownIsReference;
}

/**
 * @brief Gives the vector of movement and act over the motors
 *
 * @param Angle uint8_t. Angle of the servos in degrees or percentajes (more percentaje or angle = more elevated)
 *
 * @retval None
 */
void geometry::SetGeometry(uint8_t Angle)
{
  SetGeometry(Angle, 0);
}

/**
 * @brief Gives the vector of movement and act over the motors
 *
 * @param Angle uint8_t. Angle of the servos in degrees or percentajes (more percentaje or angle = more elevated)
 * @param Speed uint32_t. Delay in ms that takes the servos reach the position
 *
 * @retval None
 */
void geometry::SetGeometry(uint8_t Angle, uint32_t Delay)
{
  SetGeometryFront(Angle, Delay);
  SetGeometryBack(Angle, Delay);
}

/**
 * @brief Gives the vector of movement and act over the motors
 *
 * @param Angle uint8_t. Angle of the servos from the front in degrees or percentajes (more percentaje or angle = more elevated)
 *
 * @retval None
 */
void geometry::SetGeometryFront(uint8_t Angle)
{
  SetGeometryFront(Angle, 0);
}

/**
 * @brief Gives the vector of movement and act over the motors
 *
 * @param Angle uint8_t. Angle of the servos in degrees or percentajes (more percentaje or angle = more elevated)
 * @param Speed uint32_t. Delay in ms that takes the servos reach the position
 *
 * @retval None
 */
void geometry::SetGeometryFront(uint8_t Angle, uint32_t Delay)
{

  if (Use_Angle_Percentaje)
    Angle = Angle * Restringed_Front_travel / 100; // Pasar angulo a grados en funcion del recorrido que pueden hacer

  bool decrease = false;
  if (Angle < Servos_Angle_Front) // Determina la direccion
    decrease = true;

  int32_t A1_Motor_Speed, B1_Motor_Speed, A2_Motor_Speed, B2_Motor_Speed;
  A1_Motor_Speed = Compresion->GetMotorSpeed(A1_Motor);
  B1_Motor_Speed = Compresion->GetMotorSpeed(B1_Motor);
  A2_Motor_Speed = Compresion->GetMotorSpeed(A2_Motor);
  B2_Motor_Speed = Compresion->GetMotorSpeed(B2_Motor);
  if (decrease)
  {
    // Añade un factor de descompresion de 1/3 de la velocidad
    A1_Motor_Speed += A1_Motor_Speed / 3;
    B1_Motor_Speed += B1_Motor_Speed / 3;
    A2_Motor_Speed -= A2_Motor_Speed / 3;
    B2_Motor_Speed -= B2_Motor_Speed / 3;
  }
  else
  {
    // Añade un factor de compresion de 1/3 de la velocidad
    A1_Motor_Speed -= A1_Motor_Speed / 3;
    B1_Motor_Speed -= B1_Motor_Speed / 3;
    A2_Motor_Speed += A2_Motor_Speed / 3;
    B2_Motor_Speed += B2_Motor_Speed / 3;
  }
  Compresion->SetIndividualMotorSpeed(A1_Motor, A1_Motor_Speed, B1_Motor, B1_Motor_Speed, A2_Motor, A2_Motor_Speed, B2_Motor, B2_Motor_Speed);

  // Recorrer los grados correspondientes
  if (Angle < Restringed_Front_travel && Angle > 0)
  {
    for (int i = 0; i < abs(Angle - Servos_Angle_Front); i++)
    {
      if (((Servo_1->Up_End_Angle > Servo_1->Down_End_Angle) && !decrease) || ((Servo_1->Up_End_Angle < Servo_1->Down_End_Angle) && decrease)) // Entonces reduce el angulo
      {
        Servo_1_RealAngle -= i;
        variwheelerControl->setPWM(Servo_1->Pin, 0, Servo_1_RealAngle * Degree_To_Pulse_Factor + Degree_To_Pulse_Offset);
      }
      else if (((Servo_1->Up_End_Angle < Servo_1->Down_End_Angle) && !decrease) || ((Servo_1->Up_End_Angle > Servo_1->Down_End_Angle) && decrease))
      {
        Servo_1_RealAngle += i;
        variwheelerControl->setPWM(Servo_1->Pin, 0, Servo_1_RealAngle * Degree_To_Pulse_Factor + Degree_To_Pulse_Offset);
      }

      if (((Servo_2->Up_End_Angle > Servo_2->Down_End_Angle) && !decrease) || ((Servo_2->Up_End_Angle < Servo_2->Down_End_Angle) && decrease)) // Entonces reduce el angulo
      {
        Servo_2_RealAngle -= i;
        variwheelerControl->setPWM(Servo_2->Pin, 0, Servo_2_RealAngle * Degree_To_Pulse_Factor + Degree_To_Pulse_Offset);
      }
      else if (((Servo_2->Up_End_Angle < Servo_2->Down_End_Angle) && !decrease) || ((Servo_2->Up_End_Angle > Servo_2->Down_End_Angle) && decrease))
      {
        Servo_2_RealAngle += i;
        variwheelerControl->setPWM(Servo_2->Pin, 0, Servo_2_RealAngle * Degree_To_Pulse_Factor + Degree_To_Pulse_Offset);
      }

      delay(Delay / abs(Angle - Servos_Angle_Front));
    }
    Servos_Angle_Front = Angle;
  }
  else
  {
    // TODO: Todavia no hay reporte de errores en esta version
  }
}

/**
 * @brief Gives the vector of movement and act over the motors
 *
 * @param Angle uint8_t. Angle of the servos from the back in degrees or percentajes (more percentaje or angle = more elevated)
 *
 * @retval None
 */
void geometry::SetGeometryBack(uint8_t Angle)
{
  SetGeometryBack(Angle, 0);
}

/**
 * @brief Gives the vector of movement and act over the motors
 *
 * @param Angle uint8_t. Angle of the servos in degrees or percentajes (more percentaje or angle = more elevated)
 * @param Speed uint32_t. Delay in ms that takes the servos reach the position
 *
 * @retval None
 */
void geometry::SetGeometryBack(uint8_t Angle, uint32_t Delay)
{
  if (Use_Angle_Percentaje)
    Angle = Angle * Restringed_Back_travel / 100; // Pasar angulo a grados en funcion del recorrido que pueden hacer

  bool decrease = false;
  if (Angle < Servos_Angle_Back) // Determina la direccion
    decrease = true;

  int32_t A1_Motor_Speed, B1_Motor_Speed, A2_Motor_Speed, B2_Motor_Speed;
  A1_Motor_Speed = Compresion->GetMotorSpeed(A1_Motor);
  B1_Motor_Speed = Compresion->GetMotorSpeed(B1_Motor);
  A2_Motor_Speed = Compresion->GetMotorSpeed(A2_Motor);
  B2_Motor_Speed = Compresion->GetMotorSpeed(B2_Motor);
  if (decrease)
  {
    // Añade un factor de descompresion de 1/3 de la velocidad
    A1_Motor_Speed += A1_Motor_Speed / 3;
    B1_Motor_Speed += B1_Motor_Speed / 3;
    A2_Motor_Speed -= A2_Motor_Speed / 3;
    B2_Motor_Speed -= B2_Motor_Speed / 3;
  }
  else
  {
    // Añade un factor de compresion de 1/3 de la velocidad
    A1_Motor_Speed -= A1_Motor_Speed / 3;
    B1_Motor_Speed -= B1_Motor_Speed / 3;
    A2_Motor_Speed += A2_Motor_Speed / 3;
    B2_Motor_Speed += B2_Motor_Speed / 3;
  }
  Compresion->SetIndividualMotorSpeed(A1_Motor, A1_Motor_Speed, B1_Motor, B1_Motor_Speed, A2_Motor, A2_Motor_Speed, B2_Motor, B2_Motor_Speed);

  // Recorrer los grados correspondientes
  if (Angle < Restringed_Back_travel && Angle > 0)
  {
    for (int i = 0; i < abs(Angle - Servos_Angle_Back); i++)
    {
      if (((Servo_3->Up_End_Angle > Servo_3->Down_End_Angle) && !decrease) || ((Servo_3->Up_End_Angle < Servo_3->Down_End_Angle) && decrease)) // Entonces reduce el angulo
      {
        Servo_3_RealAngle -= i;
        variwheelerControl->setPWM(Servo_3->Pin, 0, Servo_3_RealAngle * Degree_To_Pulse_Factor + Degree_To_Pulse_Offset);
      }
      else if (((Servo_3->Up_End_Angle < Servo_3->Down_End_Angle) && !decrease) || ((Servo_3->Up_End_Angle > Servo_3->Down_End_Angle) && decrease))
      {
        Servo_3_RealAngle += i;
        variwheelerControl->setPWM(Servo_3->Pin, 0, Servo_3_RealAngle * Degree_To_Pulse_Factor + Degree_To_Pulse_Offset);
      }

      if (((Servo_4->Up_End_Angle > Servo_4->Down_End_Angle) && !decrease) || ((Servo_4->Up_End_Angle < Servo_4->Down_End_Angle) && decrease)) // Entonces reduce el angulo
      {
        Servo_4_RealAngle -= i;
        variwheelerControl->setPWM(Servo_4->Pin, 0, Servo_4_RealAngle * Degree_To_Pulse_Factor + Degree_To_Pulse_Offset);
      }
      else if (((Servo_4->Up_End_Angle < Servo_4->Down_End_Angle) && !decrease) || ((Servo_4->Up_End_Angle > Servo_4->Down_End_Angle) && decrease))
      {
        Servo_4_RealAngle += i;
        variwheelerControl->setPWM(Servo_4->Pin, 0, Servo_4_RealAngle * Degree_To_Pulse_Factor + Degree_To_Pulse_Offset);
      }

      delay(Delay / abs(Angle - Servos_Angle_Back));
    }
    Servos_Angle_Back = Angle;
  }
  else
  {
    // TODO: Todavia no hay reporte de errores en esta version
  }
}

/**
 * @brief Gives the vector of movement and act over the motors
 *
 * @return Angle of the front servos in degrees or percentajes
 * @retval uint8_t value
 */
uint8_t geometry::GetGeometryFront()
{
  return Servos_Angle_Front;
}

/**
 * @brief Gives the vector of movement and act over the motors
 *
 * @return Angle of the back servos in degrees or percentajes
 * @retval uint8_t value
 */
uint8_t geometry::GetGeometryBack()
{
  return Servos_Angle_Back;
}