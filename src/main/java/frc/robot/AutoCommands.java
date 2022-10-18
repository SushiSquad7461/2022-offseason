package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import SushiFrcLib.Math.Conversion;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootFender;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.Swerve;

public class AutoCommands {
    //subsystems
    private final Swerve swerve;

    //command groups for autos
    public final SequentialCommandGroup oneBall;

    public final SequentialCommandGroup nothing;

    public final SequentialCommandGroup back;

    public final SequentialCommandGroup complex;
    public final SequentialCommandGroup hubToHp;

    //names of pathplanner paths for autos
    private final String[] oneBallPaths = {"Test2"};
    private final String[] testPaths = {"Test", "Test2"};

    public AutoCommands(Swerve swerve) {
        this.swerve = swerve;

        nothing = new SequentialCommandGroup();

        oneBall = new SequentialCommandGroup(
            new InstantCommand(() -> swerve.resetOdometry(
                new Pose2d(
                    new Translation2d(0,0),
                    new Rotation2d(Conversion.degreesToRadians(180))   
                )
            )),
            new ShootFender(30000, 2300.0),
            new WaitCommand(9),
            new RunCommand(() -> swerve.drive(new Translation2d(-0.4, 0), 0, true, true), swerve)
        );

        back = new SequentialCommandGroup(getCommand(testPaths[1], true));

        complex = new SequentialCommandGroup(getCommand(testPaths[1], true), getCommand(testPaths[0], false));

        hubToHp = new SequentialCommandGroup(getCommand("HubToHp", true));
    }

    private Command getCommand(String pathName, boolean isFirstPath) {
        PathPlannerTrajectory path = PathPlanner.loadPath(pathName, Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAcceleration);
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                if (isFirstPath)
                    swerve.resetOdometry(getInitialPose(path));
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

    private Pose2d getInitialPose(PathPlannerTrajectory path) {
        return new Pose2d(path.getInitialPose().getTranslation(), path.getInitialState().holonomicRotation);
    }
}

