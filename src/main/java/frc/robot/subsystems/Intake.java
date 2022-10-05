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

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  TalonFX intakeMotor;
  TalonFX HopperMotor;

  private static Intake mInstance;

  public static Intake getInstance() {
    if (mInstance == null) {
      mInstance = new Intake();
    }
    return mInstance;
  }

  public Intake() {

    intakeMotor = MotorHelper.createFalconMotor(Constants.Ports.INTAKE_MOTOR, Constants.kIntake.CURRENT_LIMIT,
        Constants.kIntake.INTAKE_INVERSION, NeutralMode.Coast);
    HopperMotor = MotorHelper.createFalconMotor(Constants.Ports.HOPPER_MOTOR, Constants.kIntake.CURRENT_LIMIT,
        Constants.kIntake.HOPPER_INVERSION, NeutralMode.Coast);

  }

  public void runIntake() {
    if (!Indexer.getInstance().canIntake()) {
      return;
    }

    intakeMotor.set(ControlMode.PercentOutput, Constants.kIntake.INTAKE_SPEED);
    HopperMotor.set(ControlMode.PercentOutput, Constants.kIntake.HOPPER_SPEED);
  }

  public void stopIntake() {
    intakeMotor.set(ControlMode.PercentOutput, 0);
    HopperMotor.set(ControlMode.PercentOutput, 0);
  }

  public void ejectIntake() {
    intakeMotor.set(ControlMode.PercentOutput, Constants.kIntake.INTAKE_SPEED * -1);
    HopperMotor.set(ControlMode.PercentOutput, Constants.kIntake.HOPPER_SPEED * -1);
  }

  @Override
  public void periodic() {
    if (!Indexer.getInstance().canIntake()) {
      stopIntake();
    }
  }
}
