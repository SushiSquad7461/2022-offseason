// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Constants.kShooter;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Indexer.IndexerState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TeleopShoot extends PIDCommand {
  /** Creates a new Autoshoot. */

  private final Shooter m_shooter;
  private final PhotonVision m_photonvision;
  private final Swerve m_swerve;
  private final Hood m_hood;
  private final Indexer m_indexer;
  private boolean shoot = false;
  private double finishDelay = 0.0;

  public TeleopShoot() {
    super(
        // The controller that the command will use
        new PIDController(0.1, 0, 0),
        // This should return the measurement
        PhotonVision.getInstance()::getHeading,
        // This should return the setpoint (can also be a constant)
        Constants.kShooter.TX_OFFSET,
        // This uses the output
        output -> Swerve.getInstance().drive(new Translation2d(0, 0), output, false, true), 
        Swerve.getInstance());
    m_shooter = Shooter.getInstance();
    m_photonvision = PhotonVision.getInstance();
    m_swerve = Swerve.getInstance();
    m_hood = Hood.getInstance();
    m_indexer = Indexer.getInstance();
    // addRequirements(m_photonvision);
    addRequirements(m_swerve);
    addRequirements(m_shooter);
    addRequirements(m_hood);
    getController().enableContinuousInput(-180, 180);
    getController().setTolerance(kShooter.PID_TOLERANCE_DEGREES, kShooter.PID_SPEED_TOLERANCE_DEGREES_PER_SECOND);
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
    // TODO Auto-generated method stub
    super.execute();
    double distance = m_photonvision.getDistance();
    m_shooter.setVelocityBasedOnDistance(distance);
    m_hood.setPosBasedOnDistance(distance);
    if (m_shooter.isAtSpeed() && m_hood.isAtPos() && !shoot && getController().atSetpoint()) {
      m_indexer.setShooting();
      shoot = true;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isFinished = shoot && !m_indexer.getShooting();
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
    m_indexer.setState(IndexerState.IDLE);
  }
}
