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
import frc.robot.Constants.kPorts;
import frc.robot.Constants.kShooter;

public class Shooter extends SubsystemBase {
  private final WPI_TalonFX leftMotor;
  private final WPI_TalonFX rightMotor;

  private final TunableNumber shooterP;
  private final TunableNumber shooterI;
  private final TunableNumber shooterD;
  private final TunableNumber shooterF;
  private final TunableNumber shooterRPM;

  private static Shooter instance;

  public static Shooter getInstance() {
    if (instance == null) {
      instance = new Shooter();
    }
    return instance;
  }

  private Shooter() {
    shooterP = new TunableNumber("Shooter P", kShooter.kP, Constants.TUNING_MODE);
    shooterI = new TunableNumber("Shooter I", kShooter.kI, Constants.TUNING_MODE);
    shooterD = new TunableNumber("Shooter D", kShooter.kD, Constants.TUNING_MODE);
    shooterF = new TunableNumber("Shooter F", kShooter.kF, Constants.TUNING_MODE);
    shooterRPM = new TunableNumber("shooter rpm", 0, Constants.TUNING_MODE);

    leftMotor = MotorHelper.createFalconMotor(kPorts.SHOOTER_LEFT_MOTOR, kShooter.CURRENT_LIMIT,
        TalonFXInvertType.Clockwise, NeutralMode.Coast, shooterP.get(), shooterI.get(), shooterD.get(),
        shooterF.get());
    rightMotor = MotorHelper.createFalconMotor(kPorts.SHOOTER_RIGHT_MOTOR, kShooter.CURRENT_LIMIT,
        TalonFXInvertType.OpposeMaster, NeutralMode.Coast, shooterP.get(), shooterI.get(), shooterD.get(),
        shooterF.get());

    rightMotor.follow(leftMotor);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter current rpm",
        Conversion.convertTransToRPM(leftMotor.getSelectedSensorVelocity()));
    SmartDashboard.putNumber("Shooter set rpm", shooterRPM.get());
    SmartDashboard.putNumber("Shooter error", getError());

    runShooter();

    if (shooterF.hasChanged()) {
      leftMotor.config_kF(0, shooterF.get());
      rightMotor.config_kF(0, shooterF.get());
    }

    if (shooterP.hasChanged()) {
      leftMotor.config_kP(0, shooterP.get());
      rightMotor.config_kP(0, shooterF.get());
    }

    if (shooterI.hasChanged()) {
      leftMotor.config_kI(0, shooterI.get());
      rightMotor.config_kI(0, shooterF.get());
    }

    if (shooterD.hasChanged()) {
      leftMotor.config_kD(0, shooterD.get());
      rightMotor.config_kD(0, shooterF.get());
    }
  }

  public void runShooter() {
    leftMotor.set(ControlMode.Velocity, Conversion.convertRPMtoTrans(shooterRPM.get()));
  }

  public void stopShooter() {
    shooterRPM.setDefault(0);
  }

  public boolean isAtSpeed() {
    return shooterRPM.get() != 0 && getError() <= kShooter.ERROR_TOLERANCE;
  }

  private double getError() {
    return Math.abs(shooterRPM.get() - Conversion.convertTransToRPM(leftMotor.getSelectedSensorVelocity()));
  }

  public double getLeftMotorPosition() {
    return leftMotor.getSelectedSensorPosition();
  }

  public double getRightMotorPosition() {
    return rightMotor.getSelectedSensorPosition();
  }

  public void setVelocity(double speed) {
    shooterRPM.setDefault(speed);
  }

  public void setVelocityBasedOnDistance(double distance) {
    shooterRPM.setDefault(kShooter.posMap.getInterpolated(new InterpolatingDouble(distance)).value + kShooter.kOffset);
  }
}
