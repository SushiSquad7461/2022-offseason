// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import SushiFrcLib.Math.Conversion;
import SushiFrcLib.Motor.MotorHelper;
import SushiFrcLib.SmartDashboard.TunableNumber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final WPI_TalonFX leftMotor;
  private final WPI_TalonFX rightMotor;
  public final double setPointRPM;

  private TunableNumber shooterP = new TunableNumber("Shooter P", Constants.kShooter.kP, Constants.TUNING_MODE);
  private TunableNumber shooterI = new TunableNumber("Shooter I", Constants.kShooter.kI, Constants.TUNING_MODE);
  private TunableNumber shooterD = new TunableNumber("Shooter D", Constants.kShooter.kD, Constants.TUNING_MODE);
  private TunableNumber shooterF = new TunableNumber("Shooter F", Constants.kShooter.kF, Constants.TUNING_MODE);

  private TunableNumber shooterRPM = new TunableNumber("shooter rpm", Constants.kShooter.kF, Constants.TUNING_MODE);

  /** Creates a new Shooter. */
  public Shooter() {
    leftMotor = MotorHelper.createFalconMotor(Constants.Ports.SHOOTER_LEFT_MOTOR, Constants.kShooter.CURRENT_LIMIT,
        TalonFXInvertType.Clockwise, NeutralMode.Coast, shooterP.get(), shooterI.get(), shooterD.get(),
        shooterF.get());
    rightMotor = MotorHelper.createFalconMotor(Constants.Ports.SHOOTER_RIGHT_MOTOR, Constants.kShooter.CURRENT_LIMIT,
        TalonFXInvertType.CounterClockwise, NeutralMode.Coast, shooterP.get(), shooterI.get(), shooterD.get(),
        shooterF.get());

    rightMotor.follow(leftMotor);

    setPointRPM = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter current rpm",
        Conversion.convertTransToRPM(leftMotor.getSelectedSensorVelocity()));
    SmartDashboard.putNumber("Shooter set rpm", setPointRPM);
    SmartDashboard.putNumber("Shooter error", getError());
    SmartDashboard.putNumber("Left motor current draw", leftMotor.getStatorCurrent());
    SmartDashboard.putNumber("Right motor current draw", rightMotor.getStatorCurrent());

    runShooter();
    
    if (shooterF.hasChanged()) {
      leftMotor.config_kF(0, shooterF.get());
    }

    if (shooterP.hasChanged()) {
      leftMotor.config_kP(0, shooterF.get());
    }

    if (shooterI.hasChanged()) {
      leftMotor.config_kF(0, shooterI.get());
    }

    if (shooterD.hasChanged()) {
      leftMotor.config_kF(0, shooterD.get());
    }

    leftMotor.set(ControlMode.Velocity, shooterRPM.get());
    // leftMotor.set(ControlMode.PercentOutput, 0.5);
  }

  public void setMotorSpeed(ControlMode controlMode, double speed) {
    leftMotor.set(controlMode, speed);
  }

  public void runShooter() {
    setMotorSpeed(ControlMode.Velocity, setPointRPM);
  }

  public void stopShooter() {
    setMotorSpeed(ControlMode.PercentOutput, 0);
  }

  public boolean isAtSpeed() {
    return getError() <= Constants.kShooter.ERROR_TOLERANCE;
  }

  private double getError() {
    return Math.abs(Conversion.convertTransToRPM(leftMotor.getSelectedSensorVelocity()) - setPointRPM);
  }

  public double getLeftMotorPosition() {
    return leftMotor.getSelectedSensorPosition();
  }

  public double getRightMotorPosition() {
    return rightMotor.getSelectedSensorPosition();
  }
}
