// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import SushiFrcLib.Math.Normalization;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.kShooter;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Indexer.IndexerState;

public class TeleopShoot extends CommandBase {
  private final GenericHID m_controller;
  private final int m_translationAxis;
  private final int m_strafeAxis;
  private final int m_rotationsAxis;
  private final boolean m_fieldRelative;
  private final boolean m_openLoop;

  private final PIDController pid;

  private final Shooter m_shooter;
  private final PhotonVision m_photonvision;
  private final Swerve m_swerve;
  private final Hood m_hood;
  private final Indexer m_indexer;
  private boolean shoot = false;
  private double finishDelay;
  private double distance;
  private double heading;

  public TeleopShoot(GenericHID controller, int translationAxis, int strafeAxis, int rotationsAxis,
      boolean fieldRelative, boolean openLoop) {

    pid = new PIDController(0.1, 0, 0);

    finishDelay = 0.0;
    distance = 0;
    heading = 0;

    m_controller = controller;
    m_translationAxis = translationAxis;
    m_strafeAxis = strafeAxis;
    m_rotationsAxis = rotationsAxis;
    m_fieldRelative = fieldRelative;
    m_openLoop = openLoop;

    m_shooter = Shooter.getInstance();
    m_photonvision = PhotonVision.getInstance();
    m_swerve = Swerve.getInstance();
    m_hood = Hood.getInstance();
    m_indexer = Indexer.getInstance();

    addRequirements(m_photonvision);
    addRequirements(m_swerve);
    addRequirements(m_shooter);
    addRequirements(m_hood);

    pid.setSetpoint(0);
    pid.enableContinuousInput(-180, 180);
    pid.setTolerance(kShooter.PID_TOLERANCE_DEGREES);
  }

  @Override
  public void initialize() {
    shoot = false;
    finishDelay = 0.0;
    distance = m_photonvision.getDistance();
    heading = m_photonvision.getHeading();
    pid.calculate(30.0);
    SmartDashboard.putNumber("Turn To Target PID Error", pid.getPositionError());

  }

  @Override
  public void execute() {
    SmartDashboard.putBoolean("TT at Setpoint", pid.atSetpoint());
    SmartDashboard.putNumber("Turn To Target PID Error", pid.getPositionError());

    if (!pid.atSetpoint()) {
      distance = m_photonvision.getDistance();
      heading = m_photonvision.getHeading();
    } else {
      heading = 0;
    }

    double output = pid.calculate(heading);
    System.out.println(pid.atSetpoint());

    double forwardBack = -m_controller.getRawAxis(m_translationAxis);
    double leftRight = m_controller.getRawAxis(m_strafeAxis);

    forwardBack = Normalization.cube(forwardBack);
    leftRight = Normalization.cube(leftRight);

    double magnitude = new Vector2d(forwardBack, leftRight).magnitude();
    double magnitudeRatio = magnitude == 0 ? 1 : Normalization.cube(magnitude) / magnitude;
    Translation2d translation = new Translation2d(forwardBack, leftRight)
        .times(Constants.Swerve.maxSpeed * magnitudeRatio);

    m_swerve.drive(translation, output, m_fieldRelative, m_openLoop);

    m_shooter.setVelocityBasedOnDistance(distance);
    m_hood.setPosBasedOnDistance(distance);
    if (m_shooter.isAtSpeed() && m_hood.isAtPos() && !shoot && pid.atSetpoint()) {
      m_indexer.setState(IndexerState.SHOOTING);
      shoot = true;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Normalization.cube(m_controller.getRawAxis(m_rotationsAxis)) != 0) {
      return true;
    }

    boolean isFinished = shoot;
    if (isFinished) {
      if (finishDelay == 0) {
        finishDelay = Timer.getFPGATimestamp();
        return false;
      } else {
        return Timer.getFPGATimestamp() - finishDelay > 1;
      }
    }
    return isFinished;
  }

  @Override
  public void end(boolean inturrupted) {
    m_shooter.stopShooter();
    m_hood.setPos(-1000);
    m_indexer.setState(IndexerState.IDLE);
  }
}
