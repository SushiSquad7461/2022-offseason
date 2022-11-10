// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import SushiFrcLib.Math.Conversion;
import SushiFrcLib.Motor.MotorHelper;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kPorts;
import frc.robot.Constants.kIntake;;

public class Intake extends SubsystemBase {
  private final WPI_TalonFX intakeMotor;
  private final WPI_TalonFX HopperMotor;
  private boolean toggleIntake;

  private static Intake instance;

  public static Intake getInstance() {
    if (instance == null) {
      instance = new Intake();
    }
    return instance;
  }

  public Intake() {
    intakeMotor = MotorHelper.createFalconMotor(kPorts.INTAKE_MOTOR, kIntake.CURRENT_LIMIT,
        kIntake.INTAKE_INVERSION, NeutralMode.Coast);
    intakeMotor.config_kP(0, 0.1);
    HopperMotor = MotorHelper.createFalconMotor(kPorts.HOPPER_MOTOR, kIntake.CURRENT_LIMIT,
        kIntake.HOPPER_INVERSION, NeutralMode.Coast);
    toggleIntake = false;
  }

  public void runIntake() {
    if (!Indexer.getInstance().canIntake()) {
      return;
    }

    intakeMotor.set(ControlMode.Velocity, Conversion.convertRPMtoTrans(4000));
    HopperMotor.set(ControlMode.PercentOutput, kIntake.HOPPER_SPEED);
  }

  public void stopIntake() {
    intakeMotor.set(ControlMode.Velocity, 0);
    HopperMotor.set(ControlMode.PercentOutput, 0);
  }

  public void ejectIntake() {
    intakeMotor.set(ControlMode.Velocity, Conversion.convertRPMtoTrans(-4000));
    HopperMotor.set(ControlMode.PercentOutput, kIntake.HOPPER_SPEED * -1);
  }

  public void toggleIntake() {
    toggleIntake = !toggleIntake;
    if(toggleIntake) {
      runIntake();
    } else {
      stopIntake();
    }
  }

  public boolean isToggled() {
    return toggleIntake;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Speed", intakeMotor.getSelectedSensorVelocity());

    if (!Indexer.getInstance().canIntake()) {
      stopIntake();
    }
  }
}
