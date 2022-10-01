// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Constants.kShooter;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Autoshoot extends PIDCommand {
  /** Creates a new Autoshoot. */

  private static final Shooter m_shooter = Shooter.getInstance();
  private static final PhotonVision m_photonvision = PhotonVision.getInstance();
  private static final Swerve m_swerve = Swerve.getInstance();

  public Autoshoot() {
    super(
        // The controller that the command will use
        new PIDController(0.1, 0, 0),
        // This should return the measurement
        m_photonvision.getInstance()::getHeading,
        // This should return the setpoint (can also be a constant)
        Constants.kShooter.TX_OFFSET,
        // This uses the output
        output -> m_swerve.drive(new Translation2d(0, 0), output, false, true), // TODO: change drive to
                                                                                // closed loop.
        // This requires subsystems
        m_swerve);
    SmartDashboard.putBoolean("thing", true);

    // addRequirements(m_photonvision);
    addRequirements(m_swerve);
    addRequirements(m_shooter);

    getController().enableContinuousInput(-180, 180);
    getController().setTolerance(kShooter.PID_TOLERANCE_DEGREES, kShooter.PID_SPEED_TOLERANCE_DEGREES_PER_SECOND);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
