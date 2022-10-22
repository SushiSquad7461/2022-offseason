package frc.robot.commands;

import SushiFrcLib.Math.Normalization;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.kSwerve;
import frc.robot.subsystems.Swerve;

public class TeleopSwerveDrive extends CommandBase {
    private final Swerve swerve;
    private final GenericHID controller;
    private final int translationAxis;
    private final int strafeAxis;
    private final int rotationsAxis;
    private final boolean fieldRelative;
    private final boolean openLoop;

    public TeleopSwerveDrive(Swerve swerve, GenericHID controller, int translationAxis, int strafeAxis,
            int rotationsAxis, boolean fieldRelative, boolean openLoop) {
        this.swerve = swerve;
        this.controller = controller;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.rotationsAxis = rotationsAxis;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double forwardBack = -controller.getRawAxis(translationAxis);
        double leftRight = -controller.getRawAxis(strafeAxis);
        double rot = -controller.getRawAxis(rotationsAxis);

        forwardBack = Normalization.cube(forwardBack);
        leftRight = Normalization.cube(leftRight);

        double magnitude = new Vector2d(forwardBack, leftRight).magnitude();
        double magnitudeRatio = magnitude == 0 ? 1 : Normalization.cube(magnitude) / magnitude;
        Translation2d translation = new Translation2d(forwardBack, leftRight)
                .times(kSwerve.MAX_SPEED * magnitudeRatio);

        rot = Normalization.cube(Normalization.cube(rot));
        rot *= kSwerve.MAX_ANGULAR_VELOCITY;

        swerve.drive(translation, rot, fieldRelative, openLoop);
    }
}
