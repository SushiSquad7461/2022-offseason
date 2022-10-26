// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import SushiFrcLib.Constants.SushiConstants;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Indexer.IndexerState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
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
    private final Swerve swerveDrive = Swerve.getInstance();
    private final SendableChooser<SequentialCommandGroup> autoChooser;
    private final AutoCommands autos;

    private final Hood hood = Hood.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Indexer mIndexer = Indexer.getInstance();
    private final Intake mIntake = Intake.getInstance();
    private final Climb climb = Climb.getInstance();

    private final XboxController driver = new XboxController(SushiConstants.OI.DRIVER_PORT);
    private final XboxController op = new XboxController(1);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        autos = new AutoCommands(swerveDrive);
        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("Nothing", autos.nothing);
        autoChooser.addOption("One Ball", autos.oneBall);
        autoChooser.addOption("Back", autos.back);
        autoChooser.addOption("Complex", autos.complex);
        SmartDashboard.putData("Auto Selector", autoChooser);
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
                XboxController.Axis.kRightY.value,
                XboxController.Axis.kRightX.value,
                XboxController.Axis.kLeftX.value,
                true,
                false));

        // new JoystickButton(driver, XboxController.Button.kLeftBumper.value)
        // .whenPressed(new InstantCommand(swerveDrive::zeroGyro));

        new JoystickButton(driver, XboxController.Button.kLeftBumper.value)
                .whenPressed(new ParallelCommandGroup(new InstantCommand(mIntake::runIntake, mIntake),
                        new InstantCommand(mIndexer::setIntake, mIndexer)))
                .whenReleased(new SequentialCommandGroup(new InstantCommand(mIntake::stopIntake, mIntake),
                        new WaitCommand(0.5), new InstantCommand(mIndexer::setIdle, mIndexer)));

        new JoystickButton(driver, XboxController.Button.kX.value)
                .whenPressed(new InstantCommand(mIntake::ejectIntake, mIntake))
                .whenReleased(new InstantCommand(mIntake::stopIntake, mIntake));

        new JoystickButton(op, XboxController.Button.kA.value)
                .whenPressed(new InstantCommand(swerveDrive::zeroGyro));

        // new JoystickButton(driver, XboxController.Button.kRightBumper.value)
        // .whenPressed(new InstantCommand(mIndexer::setShooting, mIndexer));

        // hahaha ur having a stroke, if the turn to target doesnt work use ur big brain
        // and comment this and uncoment auto shoot 🤡🤡🤡🤡🤡🤡🤡🤡
        new JoystickButton(driver, XboxController.Button.kB.value)
                .whenPressed(new TeleopShoot(driver,
                        XboxController.Axis.kLeftY.value,
                        XboxController.Axis.kLeftX.value,
                        XboxController.Axis.kRightX.value,
                        true,
                        false));
        //

        new JoystickButton(driver, XboxController.Button.kRightBumper.value)
                .whenPressed(new Shoot(0.0, 2300.0));
        // .whenReleased(new InstantCommand(shooter::stopShooter, shooter));

        // new JoystickButton(driver, XboxController.Button.kB.value)
        // .whenPressed(new AutoShoot());
        // .whenReleased(new InstantCommand(shooter::stopShooter, shooter));
        // new JoystickButton(driver, XboxController.Button.kRightBumper.value)
        // .whenPressed(new InstantCommand(() -> mIndexer.setShooting(true), mIndexer));

        // new JoystickButton(driver, XboxController.Button.kB.value)
        // .whenPressed(new InstantCommand(() ->
        // mIndexer.setOverrideIdle(!mIndexer.getOverrideIdle()), mIndexer));

        // new JoystickButton(driver, XboxController.Button.kB.value)
        // .whenPressed(new ParallelCommandGroup(new InstantCommand(mIntake::runIntake,
        // mIntake), new InstantCommand(mIndexer::setIntake, mIndexer)))
        // .whenReleased(new InstantCommand(mIntake::stopIntake, mIntake));

        // new JoystickButton(driver, XboxController.Button.kB.value)
        // .whenPressed(new InstantCommand(mIndexer::setIntake, mIndexer))
        // .whenReleased(new InstantCommand(mIntake::stopIntake, mIntake));

        new JoystickButton(driver, XboxController.Button.kY.value)
                .whenPressed(new ParallelCommandGroup(
                        new InstantCommand(() -> mIndexer.setState(IndexerState.BACKING), mIndexer),
                        new InstantCommand(mIntake::ejectIntake, mIntake)))
                .whenReleased(new ParallelCommandGroup(
                        new InstantCommand(() -> mIndexer.setState(IndexerState.IDLE), mIndexer),
                        new InstantCommand(mIntake::stopIntake, mIntake)));

        new JoystickButton(driver, XboxController.Button.kA.value)
                .whenHeld(new ParallelCommandGroup(new InstantCommand(() -> hood.setPos(0), hood),
                        new InstantCommand(shooter::stopShooter, shooter)));

        new POVButton(op, 0)
                .whenPressed(() -> climb.openLoopLowerClimb())
                .whenReleased(() -> climb.stop());

        new POVButton(op, 180)
                .whenPressed(() -> climb.openLoopRaiseClimb())
                .whenReleased(() -> climb.stop());

        // nw JoystickButton(driver, XboxController.Button.kX.value)
        // .whenHeld(new InstantCommand(() -> hood.setPos(100000), hood));

        // new JoystickButton(driver, XboxController.Button.kX.value)
        // .whenHeld(new InstantCommand(() -> mIndexer.setState(IndexerState.MOVING_UP),
        // mIndexer));
    }

    public void teleopDrive() {
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        SmartDashboard.putData("Auto Selector", autoChooser);
        // An ExampleCommand will run in autonomous
        return autoChooser.getSelected();
    }
}
