// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import SushiFrcLib.Motor.MotorHelper;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kPorts;
import frc.robot.Constants.kIntake;;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  WPI_TalonFX intakeMotor;
  WPI_TalonFX HopperMotor;

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
    HopperMotor = MotorHelper.createFalconMotor(kPorts.HOPPER_MOTOR, kIntake.CURRENT_LIMIT,
        kIntake.HOPPER_INVERSION, NeutralMode.Coast);

  }

  public void runIntake() {
    intakeMotor.set(ControlMode.PercentOutput, kIntake.INTAKE_SPEED);
    HopperMotor.set(ControlMode.PercentOutput, kIntake.HOPPER_SPEED);
  }

  public void stopIntake() {
    intakeMotor.set(ControlMode.PercentOutput, 0);
    HopperMotor.set(ControlMode.PercentOutput, 0);
  }

  public void ejectIntake() {
    intakeMotor.set(ControlMode.PercentOutput, kIntake.INTAKE_SPEED * -1);
    HopperMotor.set(ControlMode.PercentOutput, kIntake.HOPPER_SPEED * -1);
  }

  @Override
  public void periodic() { }
}
