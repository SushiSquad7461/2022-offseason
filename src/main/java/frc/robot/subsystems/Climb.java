// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import SushiFrcLib.Motor.MotorHelper;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */

  TalonFX leftMotor;
  TalonFX rightMotor;

  private static Climb mInstance;

  public static Climb getInstance() {
    if (mInstance == null) {
      mInstance = new Climb();
    }
    return mInstance;
  }

  public Climb() {

    leftMotor = MotorHelper.createFalconMotor(Constants.Ports.INTAKE_MOTOR, Constants.kClimb.CURRENT_LIMIT,
        Constants.kClimb.LEFT_INVERSION, NeutralMode.Brake);
    rightMotor = MotorHelper.createFalconMotor(Constants.Ports.HOPPER_MOTOR, Constants.kClimb.CURRENT_LIMIT,
        Constants.kClimb.RIGHT_INVERSION, NeutralMode.Brake);

  }

  public void openLoopRaiseClimb() {
    leftMotor.set(ControlMode.PercentOutput, Constants.kClimb.CLIMB_SPEED);
    rightMotor.set(ControlMode.PercentOutput, Constants.kClimb.CLIMB_SPEED);
  }

  public void stopIntake() {
    leftMotor.set(ControlMode.PercentOutput, 0);
    rightMotor.set(ControlMode.PercentOutput, 0);
  }

  public void openLoopLowerClimb() {
    leftMotor.set(ControlMode.PercentOutput, Constants.kClimb.CLIMB_SPEED * -1);
    rightMotor.set(ControlMode.PercentOutput, Constants.kClimb.CLIMB_SPEED * -1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
