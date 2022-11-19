// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.kAutoShoot;
import frc.robot.Constants.kShooter;
import frc.robot.Constants.kSwerve;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Indexer.IndexerState;

public class TeleopShoot extends CommandBase {
  private final Supplier<Double> getRotation;
  private final PIDController pid;

  private final Shooter shooter;
  private final PhotonVision photonvision;
  private final Swerve swerve;
  private final Hood hood;
  private final Indexer indexer;

  private boolean shoot;
  private double finishDelay;
  private double initTime;

  public TeleopShoot(Supplier<Double> getRotation) {
    pid = new PIDController(kAutoShoot.kP, kAutoShoot.kI, kAutoShoot.kD);

    this.getRotation = getRotation;

    finishDelay = 0.0;
    shoot = false;

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
    pid.setSetpoint(0);
    initTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    double heading = photonvision.getHeading();
    double output = pid.calculate(heading * -1);

    if (pid.atSetpoint()) {
      output = 0;
    }

    SmartDashboard.putBoolean("TT at Setpoint", pid.atSetpoint());
    SmartDashboard.putNumber("Turn To Target PID Error", pid.getPositionError());
    SmartDashboard.putNumber("TT Output", output);

    swerve.drive(new Translation2d(0,0), output, true, false);

    double distance = photonvision.getDistance();
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
    if (Timer.getFPGATimestamp() - initTime > Constants.SHOOT_UP_TO_SPEED_TIMEOUT && !shoot) {
      return true;
    }
    
    if (Math.abs(getRotation.get()) > 0.3) {
      return true;
    }
    
    if (shoot) {
      if (finishDelay == 0) {
        finishDelay = Timer.getFPGATimestamp();
        return false;
      } else {
        return Timer.getFPGATimestamp() - finishDelay > 1;
      }
    }
    
    return false;
  }

  @Override
  public void end(boolean inturrupted) {
    shooter.stopShooter();
    hood.setPos(-1000);
    indexer.setState(IndexerState.IDLE);
  }
}
