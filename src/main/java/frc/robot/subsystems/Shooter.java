// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import SushiFrcLib.CheesyLibUtil.InterpolatingDouble;
import SushiFrcLib.Math.Conversion;
import SushiFrcLib.Motor.MotorHelper;
import SushiFrcLib.SmartDashboard.TunableNumber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final WPI_TalonFX leftMotor;
  private final WPI_TalonFX rightMotor;

  private final TunableNumber shooterP;
  private final TunableNumber shooterI;
  private final TunableNumber shooterD;
  private final TunableNumber shooterF;
  // private final TunableNumber shooterRPM;
  private double shooterRPM = 0;

  private static Shooter mInstance;

  public static Shooter getInstance() {
    if (mInstance == null) {
      mInstance = new Shooter();
    }
    return mInstance;
  }

  private Shooter() {
    shooterP = new TunableNumber("Shooter P", Constants.kShooter.kP, Constants.TUNING_MODE);
    shooterI = new TunableNumber("Shooter I", Constants.kShooter.kI, Constants.TUNING_MODE);
    shooterD = new TunableNumber("Shooter D", Constants.kShooter.kD, Constants.TUNING_MODE);
    shooterF = new TunableNumber("Shooter F", Constants.kShooter.kF, Constants.TUNING_MODE);
    // shooterRPM = new TunableNumber("shooter rpm", 0, Constants.TUNING_MODE);
    shooterRPM = 0;

    leftMotor = MotorHelper.createFalconMotor(Constants.Ports.SHOOTER_LEFT_MOTOR, Constants.kShooter.CURRENT_LIMIT,
        TalonFXInvertType.Clockwise, NeutralMode.Coast, shooterP.get(), shooterI.get(), shooterD.get(),
        shooterF.get());
    rightMotor = MotorHelper.createFalconMotor(Constants.Ports.SHOOTER_RIGHT_MOTOR, Constants.kShooter.CURRENT_LIMIT,
        TalonFXInvertType.CounterClockwise, NeutralMode.Coast, shooterP.get(), shooterI.get(), shooterD.get(),
        shooterF.get());

    rightMotor.follow(leftMotor);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter current rpm",
        Conversion.convertTransToRPM(leftMotor.getSelectedSensorVelocity()));
    SmartDashboard.putNumber("Shooter set rpm", shooterRPM);
    SmartDashboard.putNumber("Shooter error", getError());
    SmartDashboard.putNumber("Loop error", leftMotor.getClosedLoopError());
    // SmartDashboard.putNumber("Left motor current draw",
    // leftMotor.getStatorCurrent());
    // SmartDashboard.putNumber("Right motor current draw",
    // rightMotor.getStatorCurrent());

    runShooter();

    if (shooterF.hasChanged()) {
      leftMotor.config_kF(0, shooterF.get());
    }

    if (shooterP.hasChanged()) {
      leftMotor.config_kP(0, shooterP.get());
    }

    if (shooterI.hasChanged()) {
      leftMotor.config_kI(0, shooterI.get());
    }

    if (shooterD.hasChanged()) {
      leftMotor.config_kD(0, shooterD.get());
    }
  }

  public void runShooter() {
    leftMotor.set(ControlMode.Velocity, Conversion.convertRPMtoTrans(shooterRPM));
  }

  public void stopShooter() {
    leftMotor.set(ControlMode.PercentOutput, 0);
    shooterRPM = 0;
  }

  public boolean isAtSpeed() {
    return shooterRPM != 0 && getError() <= Constants.kShooter.ERROR_TOLERANCE;
    // return true;
  }

  private double getError() {
    return Math.abs(shooterRPM - Conversion.convertTransToRPM(leftMotor.getSelectedSensorVelocity()));
  }

  public double getLeftMotorPosition() {
    return leftMotor.getSelectedSensorPosition();
  }

  public double getRightMotorPosition() {
    return rightMotor.getSelectedSensorPosition();
  }

  public void setVelocity(double speed) {
    // shooterRPM.setDefault(speed);
    shooterRPM = speed;
  }

  public void setVelocityBasedOnDistance(double distance) {
    shooterRPM = Constants.kShooter.posMap.getInterpolated(new InterpolatingDouble(distance)).value;
  }
}
