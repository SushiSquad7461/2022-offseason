// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import SushiFrcLib.Constants.SushiConstants;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

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
  private final Swerve swerveDrive = Swerve.getInstance();
  private final SendableChooser<SequentialCommandGroup> autoChooser;
  private final AutoCommands autos;

  private final Hood hood = Hood.getInstance();
  private final Shooter shooter = Shooter.getInstance();
  private final Indexer mIndexer = Indexer.getInstance();
  private final Intake mIntake = Intake.getInstance();

  private final XboxController driver = new XboxController(SushiConstants.OI.DRIVER_PORT);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    autos = new AutoCommands(swerveDrive);
    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("Test", autos.test);

    // Configure the button bindings
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
    swerveDrive.setDefaultCommand(new SwerveDriveCommand(
      swerveDrive, 
      driver, 
      XboxController.Axis.kLeftY.value,
      XboxController.Axis.kLeftX.value, 
      XboxController.Axis.kRightX.value,
      false,
      true
    ));

    new JoystickButton(driver, XboxController.Button.kY.value)
        .whenPressed(new InstantCommand(mIndexer::setIntake, mIndexer))
        .whenReleased(new InstantCommand(mIndexer::setIdle, mIndexer));
    new JoystickButton(driver, XboxController.Button.kB.value)
        .whenPressed(new ParallelCommandGroup(new InstantCommand(mIntake::runIntake, mIntake), new InstantCommand(mIndexer::setIntake, mIntake)))
        .whenReleased(new InstantCommand(mIntake::stopIntake, mIntake));
      
    new JoystickButton(driver, XboxController.Button.kA.value)
        .whenHeld(new InstantCommand(() -> hood.setPos(0), hood));
    new JoystickButton(driver, XboxController.Button.kX.value)
        .whenHeld(new InstantCommand(() -> hood.setPos(100000), hood));
  }

  public void teleopDrive() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }
}
