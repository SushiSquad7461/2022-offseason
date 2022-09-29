// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kClimb;
import frc.robot.Constants;
import frc.robot.Constants.Ports;
import SushiFrcLib.Control.FeedForwardController;
import SushiFrcLib.Motor.MotorHelper;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */

  CANSparkMax leftMotor;
  CANSparkMax rightMotor;
  PIDController pidController;
  SimpleMotorFeedforward feedforward;
  int pidOut = 0;
  boolean closedLoop = true;

  public Climb() {
    leftMotor = MotorHelper.createSparkMax(Ports.LEFT_CLIMB_MOTOR, kClimb.MOTOR_TYPE, kClimb.LEFT_INVERSION, kClimb.CURRENT_LIMIT, kClimb.IDLE_MODE);
    rightMotor = MotorHelper.createSparkMax(Ports.RIGHT_CLIMB_MOTOR, kClimb.MOTOR_TYPE, kClimb.RIGHT_INVERSION, kClimb.CURRENT_LIMIT, kClimb.IDLE_MODE);

    rightMotor.follow(leftMotor, false);

    pidController = new PIDController(
      Constants.kClimb.kP, 
      Constants.kClimb.kI, 
      Constants.kClimb.kD
    );
    
    feedforward = new SimpleMotorFeedforward(
      Constants.kClimb.kS,
      Constants.kClimb.kV,
      Constants.kClimb.kA
    );
  }

  @Override
  public void periodic() {
    if(closedLoop) {
    }
  }

  public void stop() {
    leftMotor.set(0);
  }

  public void runUp() {
    leftMotor.set(Constants.kClimb.OPEN_LOOP_SPEED);
  }

  public void runDown() {
    leftMotor.set(-Constants.kClimb.OPEN_LOOP_SPEED);
  }
}
