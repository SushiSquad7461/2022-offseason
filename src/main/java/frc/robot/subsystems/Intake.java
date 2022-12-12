// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import SushiFrcLib.Math.Conversion;
import SushiFrcLib.Motor.MotorHelper;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kPorts;
import frc.robot.Constants.kIntake;;

public class Intake extends SubsystemBase {
  private final WPI_TalonFX intakeMotor;
  private final WPI_TalonFX hopperMotor;
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

    intakeMotor.config_kP(0, kIntake.kP);

    hopperMotor = MotorHelper.createFalconMotor(kPorts.HOPPER_MOTOR, kIntake.CURRENT_LIMIT,
        kIntake.HOPPER_INVERSION, NeutralMode.Coast);

    toggleIntake = false;
  }

  public void runIntake() {
    setIntakeMotor(ControlMode.Velocity, Conversion.convertRPMtoTrans(kIntake.INTAKE_SPEED));
    setHopperMotor(ControlMode.PercentOutput, kIntake.HOPPER_SPEED);
  }

  public void stopIntake() {
    setIntakeMotor(ControlMode.Velocity, 0);
    setHopperMotor(ControlMode.PercentOutput, 0);
  }

  public void ejectIntake() {
    setIntakeMotor(ControlMode.Velocity, Conversion.convertRPMtoTrans(kIntake.EJECT_INTAKE_SPEED));
    setHopperMotor(ControlMode.PercentOutput, kIntake.EJECT_HOPPER_SPEED);
  }

  private void setIntakeMotor(ControlMode controlMode, double speed) {
    intakeMotor.set(controlMode, speed);
  }

  private void setHopperMotor(ControlMode controlMode, double speed) {
    hopperMotor.set(controlMode, speed);
  }

  public void toggleIntake() {
    toggleIntake = !toggleIntake;

    if (toggleIntake) {
      runIntake();
    } else {
      stopIntake();
    }
  }

  public boolean isToggled() {
    return toggleIntake;
  }
}
