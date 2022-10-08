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
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Constants.kShooter;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Indexer.IndexerState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TeleopShoot extends CommandBase {
  /** Creates a new Autoshoot. */

  private final GenericHID m_controller;
  private final int m_translationAxis;
  private final int m_strafeAxis;
  private final int m_rotationsAxis;
  private final boolean m_fieldRelative;
  private final boolean m_openLoop;

  private final PIDController pid = new PIDController(0.04, 0, 0);

  private final Shooter m_shooter;
  private final PhotonVision m_photonvision;
  private final Swerve m_swerve;
  private final Hood m_hood;
  private final Indexer m_indexer;
  private boolean shoot = false;
  private double finishDelay = 0.0;

  public TeleopShoot(GenericHID controller, int translationAxis, int strafeAxis, int rotationsAxis,
      boolean fieldRelative, boolean openLoop) {
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
    // addRequirements(m_photonvision);
    addRequirements(m_swerve);
    addRequirements(m_shooter);
    addRequirements(m_hood);

    pid.setSetpoint(0);
    pid.enableContinuousInput(-180, 180);
    pid.setTolerance(kShooter.PID_TOLERANCE_DEGREES, kShooter.PID_SPEED_TOLERANCE_DEGREES_PER_SECOND);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  @Override
  public void initialize() {
    shoot = false;
    finishDelay = 0.0;
  }

  @Override
  public void execute() {
    double output = pid.calculate(m_photonvision.getHeading());

    double forwardBack = -m_controller.getRawAxis(m_translationAxis);
    double leftRight = m_controller.getRawAxis(m_strafeAxis);

    forwardBack = Normalization.linearDeadzone(forwardBack, Constants.stickDeadband);
    leftRight = Normalization.linearDeadzone(leftRight, Constants.stickDeadband);

    double magnitude = new Vector2d(forwardBack, leftRight).magnitude();
    double magnitudeRatio = magnitude == 0 ? 1 : Normalization.cube(magnitude) / magnitude;
    Translation2d translation = new Translation2d(forwardBack, leftRight)
        .times(Constants.Swerve.maxSpeed * magnitudeRatio);

    m_swerve.drive(translation, output, m_fieldRelative, m_openLoop);

    double distance = m_photonvision.getDistance();
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
    if (Normalization.linearDeadzone(m_controller.getRawAxis(m_rotationsAxis), Constants.stickDeadband) != 0) {
      return true;
    }

    // boolean isFinished = shoot && !m_indexer.getShooting();
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
