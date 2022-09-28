package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.kSwerveDrive;
import frc.robot.subsystems.Swerve;

public class AutoCommands {
    //subsystems
    private final Swerve swerve;

    //command groups for autos
    public final SequentialCommandGroup test;

    //names of pathplanner paths for autos
    private final String[] testPaths = {"Test"};

    public AutoCommands(Swerve swerve) {
        this.swerve = swerve;

        test = new SequentialCommandGroup(
            getCommand(testPaths[0])
        );
    }

    private PPSwerveControllerCommand getCommand(String pathName) {
        PathPlannerTrajectory path = PathPlanner.loadPath(pathName, kSwerveDrive.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, kSwerveDrive.MAX_ACCELERATION);
        return new PPSwerveControllerCommand(
            path, swerve::getPose, swerve.getKinematics(), kSwerveDrive.X_CONTROLLER, kSwerveDrive.Y_CONTROLLER, kSwerveDrive.ANGLE_CONTROLLER, (s -> swerve.updateModules(s)), swerve);
    }
}

