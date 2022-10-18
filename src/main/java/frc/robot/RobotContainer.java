// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import SushiFrcLib.Constants.SushiConstants;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Indexer.IndexerState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.kOI;
import frc.robot.commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Swerve swerve;
  private final SendableChooser<SequentialCommandGroup> autoChooser;
  private final AutoCommands autos;

  private final Hood hood;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Intake intake;

  private final XboxController driver = new XboxController(SushiConstants.OI.DRIVER_PORT);
  private final XboxController op = new XboxController(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    swerve = Swerve.getInstance();
    hood = Hood.getInstance();
    shooter = Shooter.getInstance();
    indexer = Indexer.getInstance();
    intake = Intake.getInstance();
    autos = new AutoCommands(swerve);
    autoChooser = new SendableChooser<>();

    Set<String> keys = autos.autos.keySet();
    autoChooser.setDefaultOption((String) keys.toArray()[0], autos.autos.get(keys.toArray()[0]));
    keys.remove((String) keys.toArray()[0]);

    for (String i : autos.autos.keySet()) {
        autoChooser.addOption(i, autos.autos.get(i));
    }

    SmartDashboard.putData("Auto Selector", autoChooser);

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    swerve.setDefaultCommand(new SwerveDriveCommand(
            swerve,
            driver,
            kOI.DRIVE_TRANSLATION_Y,
            kOI.DRIVE_TRANSLATION_X,
            kOI.DRIVE_ROTATE,
            true,
            false
        )
    );

    new JoystickButton(driver, kOI.RUN_INTAKE)
        .whenPressed(
            new ParallelCommandGroup(
                new InstantCommand(intake::runIntake, intake),
                new InstantCommand(indexer::setIntake, indexer)
            )
        )
        .whenReleased(
            new SequentialCommandGroup(
                new InstantCommand(intake::stopIntake, intake), 
                new WaitCommand(0.5),
                new InstantCommand(indexer::setIdle, indexer)
            )
        );

    new JoystickButton(driver, kOI.EJECT_INTAKE)
        .whenPressed(new InstantCommand(intake::ejectIntake, intake))
        .whenReleased(new InstantCommand(intake::stopIntake, intake));

    new JoystickButton(op, kOI.ZERO_GYRO)
        .whenPressed(new InstantCommand(swerve::zeroGyro));

    new JoystickButton(driver, kOI.AUTO_SHOOT)
        .whenPressed(new TeleopShoot(
                driver,
                kOI.DRIVE_TRANSLATION_Y,
                kOI.DRIVE_TRANSLATION_X,
                kOI.DRIVE_ROTATE,
                true,
                false
            )
        );

    new JoystickButton(driver, kOI.FENDER_SHOOT)
        .whenPressed(new ShootFender(0.0, 2300.0));

    new JoystickButton(driver, kOI.BACK_INDEXER)
        .whenPressed(
            new ParallelCommandGroup(
                new InstantCommand(() -> indexer.setState(IndexerState.BACKING), indexer),
                new InstantCommand(intake::ejectIntake, intake)
            )
        )
        .whenReleased(
            new ParallelCommandGroup(
                new InstantCommand(() -> indexer.setState(IndexerState.IDLE), indexer),
                new InstantCommand(intake::stopIntake, intake)
            )
        );

    new JoystickButton(driver, kOI.ZERO_SHOOTER_HOOD)
        .whenHeld(
            new ParallelCommandGroup(
                new InstantCommand(() -> hood.setPos(0), hood),
                new InstantCommand(shooter::stopShooter, shooter)
            )
        );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    SmartDashboard.putData("Auto Selector", autoChooser);
    return autoChooser.getSelected();
  }
}
