// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import SushiFrcLib.Motor.MotorHelper;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final WPI_TalonFX leftMotor;
  private final WPI_TalonFX rightMotor;
  public static double setPointRPM;

  public enum ShooterState {

  }

  /** Creates a new Shooter. */
  public Shooter() {
    leftMotor = MotorHelper.createFalconMotor(Constants.Ports.SHOOTER_LEFT_MOTOR, Constants.kShooter.CURRENT_LIMIT, 
      TalonFXInvertType.Clockwise, NeutralMode.Coast, Constants.kShooter.kP, Constants.kShooter.kI, Constants.kShooter.kD, 
      Constants.kShooter.kF);
    rightMotor = MotorHelper.createFalconMotor(Constants.Ports.SHOOTER_RIGHT_MOTOR, Constants.kShooter.CURRENT_LIMIT, 
      TalonFXInvertType.CounterClockwise, NeutralMode.Coast, Constants.kShooter.kP, Constants.kShooter.kI, Constants.kShooter.kD, 
      Constants.kShooter.kF);

    // rightMotor.follow(leftMotor);

    setPointRPM = 0;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter current rpm", Constants.convertTransToRPM(leftMotor.getSelectedSensorVelocity()));
    SmartDashboard.putNumber("Shooter set rpm", setPointRPM);
    SmartDashboard.putNumber("Shooter error", getError());
    SmartDashboard.putNumber("Left motor current draw", leftMotor.getStatorCurrent());
    SmartDashboard.putNumber("Right motor current draw", rightMotor.getStatorCurrent());

    runShooter();
    setLeftMotorSpeed(ControlMode.PercentOutput, 0.25);
    setRightMotorSpeed(ControlMode.PercentOutput, 0.25);
  }

  public void setLeftMotorSpeed (ControlMode controlMode, double speed) {
    leftMotor.set(controlMode, speed);
  }

  public void setRightMotorSpeed (ControlMode controlMode, double speed) {
    rightMotor.set(controlMode, speed);
  }

  public void runShooter() {
    setLeftMotorSpeed(ControlMode.Velocity, setPointRPM);
    setRightMotorSpeed(ControlMode.Velocity, setPointRPM);
  }

  public void stopShooter() {
    setLeftMotorSpeed(ControlMode.PercentOutput, 0);
    setRightMotorSpeed(ControlMode.PercentOutput, 0);
  }

  public boolean isAtSpeed() {
    return getError() <= Constants.kShooter.ERROR_TOLERANCE;
  }

  private double getError() {
    return Math.abs(Constants.convertTransToRPM(leftMotor.getSelectedSensorVelocity()) - setPointRPM);
  }

  public double getLeftMotorPosition() {
    return leftMotor.getSelectedSensorPosition();
  }

  public double getRightMotorPosition() {
    return rightMotor.getSelectedSensorPosition();
  }
}
