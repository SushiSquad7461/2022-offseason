package frc.robot.commands;

import SushiFrcLib.Math.Normalization;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class SwerveDriveCommand extends CommandBase {
    private final Swerve m_swerve;
    private final GenericHID m_controller;
    private final int m_translationAxis;
    private final int m_strafeAxis;
    private final int m_rotationsAxis;
    private final boolean m_fieldRelative;
    private final boolean m_openLoop;

    public SwerveDriveCommand(Swerve swerve, GenericHID controller, int translationAxis, int strafeAxis, int rotationsAxis, boolean fieldRelative, boolean openLoop) {
        m_swerve = swerve;
        m_controller = controller;
        m_translationAxis = translationAxis;
        m_strafeAxis = strafeAxis;
        m_rotationsAxis = rotationsAxis;
        m_fieldRelative = fieldRelative;
        m_openLoop = openLoop;

        addRequirements(m_swerve);
    }

    @Override
    public void execute() {
        double forwardBack = -m_controller.getRawAxis(m_translationAxis);
        double leftRight = m_controller.getRawAxis(m_strafeAxis);
        double rot = m_controller.getRawAxis(m_rotationsAxis);

        forwardBack = Normalization.cube(Normalization.cube(forwardBack));
        leftRight = Normalization.cube(Normalization.cube(leftRight));
        rot = Normalization.cube(Normalization.cube(rot));

        Translation2d translation = new Translation2d(forwardBack, leftRight).times(Constants.Swerve.maxSpeed);
        rot *= Constants.Swerve.maxAngularVelocity;

        m_swerve.drive(translation, rot, m_fieldRelative, m_openLoop);
    }
}
