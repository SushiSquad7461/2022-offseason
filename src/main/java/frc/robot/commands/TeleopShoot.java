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
import frc.robot.Constants.kShooter;
import frc.robot.Constants.kSwerve;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Indexer.IndexerState;

public class TeleopShoot extends CommandBase {
  private final GenericHID controller;
  private final int translationAxis;
  private final int strafeAxis;
  private final int rotationsAxis;
  private final boolean fieldRelative;
  private final boolean openLoop;

  private final PIDController pid;

  private final Shooter shooter;
  private final PhotonVision photonvision;
  private final Swerve swerve;
  private final Hood hood;
  private final Indexer indexer;
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

    this.controller = controller;
    this.translationAxis = translationAxis;
    this.strafeAxis = strafeAxis;
    this.rotationsAxis = rotationsAxis;
    this.fieldRelative = fieldRelative;
    this.openLoop = openLoop;

    shooter = Shooter.getInstance();
    photonvision = PhotonVision.getInstance();
    swerve = Swerve.getInstance();
    hood = Hood.getInstance();
    indexer = Indexer.getInstance();

    addRequirements(photonvision);
    addRequirements(swerve);
    addRequirements(shooter);
    addRequirements(hood);

    pid.setSetpoint(0);
    pid.enableContinuousInput(-180, 180);
    pid.setTolerance(kShooter.PID_TOLERANCE_DEGREES);
  }

  @Override
  public void initialize() {
    shoot = false;
    finishDelay = 0.0;
    distance = photonvision.getDistance();
    heading = photonvision.getHeading();
    pid.calculate(30.0);
    SmartDashboard.putNumber("Turn To Target PID Error", pid.getPositionError());

  }

  @Override
  public void execute() {
    SmartDashboard.putBoolean("TT at Setpoint", pid.atSetpoint());
    SmartDashboard.putNumber("Turn To Target PID Error", pid.getPositionError());

    if (!pid.atSetpoint()) {
      distance = photonvision.getDistance();
      heading = photonvision.getHeading();
    } else {
      heading = 0;
    }

    double output = pid.calculate(heading);
    System.out.println(pid.atSetpoint());

    double forwardBack = -controller.getRawAxis(translationAxis);
    double leftRight = controller.getRawAxis(strafeAxis);

    forwardBack = Normalization.cube(forwardBack);
    leftRight = Normalization.cube(leftRight);

    double magnitude = new Vector2d(forwardBack, leftRight).magnitude();
    double magnitudeRatio = magnitude == 0 ? 1 : Normalization.cube(magnitude) / magnitude;
    Translation2d translation = new Translation2d(forwardBack, leftRight)
        .times(kSwerve.maxSpeed * magnitudeRatio);

    swerve.drive(translation, output, fieldRelative, openLoop);

    shooter.setVelocityBasedOnDistance(distance);
    hood.setPosBasedOnDistance(distance);
    if (shooter.isAtSpeed() && hood.isAtPos() && !shoot && pid.atSetpoint()) {
      indexer.setState(IndexerState.SHOOTING);
      shoot = true;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Normalization.cube(controller.getRawAxis(rotationsAxis)) != 0) {
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
    shooter.stopShooter();
    hood.setPos(-1000);
    indexer.setState(IndexerState.IDLE);
  }
}
