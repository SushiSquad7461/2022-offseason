package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

public class AutoCommands {
    //subsystems
    private final Swerve swerve;

    //command groups for autos
    public final SequentialCommandGroup test;

    public final SequentialCommandGroup back;

    //names of pathplanner paths for autos
    private final String[] testPaths = {"Test", "Test2"};

    public AutoCommands(Swerve swerve) {
        this.swerve = swerve;

        test = new SequentialCommandGroup(
            // getCommand(testPaths[0])
            getCommand(testPaths[1])
        );

        back = new SequentialCommandGroup(getCommand(testPaths[1]));
    }

    private Command getCommand(String pathName) {
        PathPlannerTrajectory path = PathPlanner.loadPath(pathName, Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAcceleration);
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                swerve.resetOdometry(path.sample(0).poseMeters);
            }, swerve),
            new PPSwerveControllerCommand(
                path, 
                swerve::getPose, 
                Constants.Swerve.swerveKinematics, 
                Constants.Swerve.xController, 
                Constants.Swerve.yController, 
                Constants.Swerve.angleController, 
                (s -> swerve.setModuleStates(s)), 
                swerve
            )
        );
    }
}

