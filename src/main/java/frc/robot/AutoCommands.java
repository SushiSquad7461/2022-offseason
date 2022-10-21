package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import SushiFrcLib.Math.Conversion;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.kShots;
import frc.robot.Constants.kSwerve;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class AutoCommands {

    //subsystems
    private final Swerve swerve;
    private final Intake intake;
    private final Indexer indexer;
    public final Map<String, SequentialCommandGroup> autos;

    //names of pathplanner paths for autos
    private final String[] sixBallPaths = {"TarmacToSide", "SideToHP", "HPToShot2", "ShotToFarBall"};
    private final String[] fiveBallPaths = {"TarmacToSide", "SideToBall", "BallToHP", "HPToShot"};
    private final String[] twoBallPaths = {"TarmacToBall"};

    public AutoCommands(Swerve swerve, Intake intake, Indexer indexer) {
        this.swerve = swerve;
        this.intake = intake;
        this.indexer = indexer;

        autos = new HashMap<String, SequentialCommandGroup>();

        autos.put("nothing", new SequentialCommandGroup());

        autos.put("1 Ball", new SequentialCommandGroup(
            new InstantCommand(() -> swerve.resetOdometry(
                new Pose2d(
                    new Translation2d(0,0),
                    new Rotation2d(Conversion.degreesToRadians(180))   
                )
            )),
            new Shoot(kShots.FENDER.hoodAngle, kShots.FENDER.shooterVelocity),
            new WaitCommand(9),
            new RunCommand(() -> swerve.drive(new Translation2d(-0.4, 0), 0, true, true), swerve)
        ));

        autos.put("2 Ball", new SequentialCommandGroup(
            new ParallelCommandGroup(
                getCommand(twoBallPaths[0], true), 
                new InstantCommand(intake::runIntake, intake),
                new InstantCommand(indexer::setIntake, indexer)
            ),
            new ParallelCommandGroup(
                new Shoot(kShots.AUTO_TARMAC.hoodAngle, kShots.AUTO_TARMAC.shooterVelocity),
                new SequentialCommandGroup(
                    new InstantCommand(intake::stopIntake, intake),
                    new WaitCommand(0.5),
                    new InstantCommand(indexer::setIdle, indexer)
                )
            )
        ));

        autos.put("3 Ball", new SequentialCommandGroup(
            new ParallelCommandGroup(
                getCommand(fiveBallPaths[0], true),
                new InstantCommand(intake::runIntake, intake),
                new InstantCommand(indexer::setIntake, indexer)
            ),
            new ParallelCommandGroup(
                new Shoot(kShots.AUTO_SIDE.hoodAngle, kShots.AUTO_SIDE.shooterVelocity),
                new SequentialCommandGroup(
                    new InstantCommand(intake::stopIntake, intake),
                    new WaitCommand(0.5),
                    new InstantCommand(indexer::setIdle, indexer)
                )
            ),
            new ParallelCommandGroup(
                getCommand(fiveBallPaths[1], false),
                new InstantCommand(intake::runIntake, intake),
                new InstantCommand(indexer::setIntake, indexer)
            ),
            new ParallelCommandGroup(
                new Shoot(kShots.AUTO_TARMAC.hoodAngle, kShots.AUTO_TARMAC.shooterVelocity),
                new SequentialCommandGroup(
                    new InstantCommand(intake::stopIntake, intake),
                    new WaitCommand(0.5),
                    new InstantCommand(indexer::setIdle, indexer)
                )
            )
        ));

        autos.put("5 Ball", new SequentialCommandGroup(
            new ParallelCommandGroup(
                getCommand(fiveBallPaths[0], true),
                new InstantCommand(intake::runIntake, intake),
                new InstantCommand(indexer::setIntake, indexer)
            ),
            new ParallelCommandGroup(
                new Shoot(kShots.AUTO_SIDE.hoodAngle, kShots.AUTO_SIDE.shooterVelocity),
                new SequentialCommandGroup(
                    new InstantCommand(intake::stopIntake, intake),
                    new WaitCommand(0.5),
                    new InstantCommand(indexer::setIdle, indexer)
                )
            ),
            new ParallelCommandGroup(
                getCommand(fiveBallPaths[1], false),
                new InstantCommand(intake::runIntake, intake),
                new InstantCommand(indexer::setIntake, indexer)
            ),
            new ParallelCommandGroup(
                new Shoot(kShots.AUTO_TARMAC.hoodAngle, kShots.AUTO_TARMAC.shooterVelocity),
                new SequentialCommandGroup(
                    new InstantCommand(intake::stopIntake, intake),
                    new WaitCommand(0.5),
                    new InstantCommand(indexer::setIdle, indexer)
                )
            ),
            new ParallelCommandGroup(
                getCommand(fiveBallPaths[2], false),
                new InstantCommand(intake::runIntake, intake),
                new InstantCommand(indexer::setIntake, indexer)
            ),
            new WaitCommand(2),
            new ParallelCommandGroup(
                getCommand(fiveBallPaths[3], false),
                new SequentialCommandGroup(
                    new InstantCommand(intake::stopIntake, intake), 
                    new WaitCommand(0.5),
                    new InstantCommand(indexer::setIdle, indexer)
                )
            ),
            new ParallelCommandGroup(
                new Shoot(kShots.AUTO_TARMAC.hoodAngle, kShots.AUTO_TARMAC.shooterVelocity),
                new SequentialCommandGroup(
                    new InstantCommand(intake::stopIntake, intake),
                    new WaitCommand(0.5),
                    new InstantCommand(indexer::setIdle, indexer)
                )
            )
        ));
        autos.put("6 Ball",
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    getCommand(fiveBallPaths[0], true),
                    new InstantCommand(intake::runIntake, intake),
                    new InstantCommand(indexer::setIntake, indexer)
                ),
                new ParallelCommandGroup(
                    new Shoot(kShots.AUTO_SIDE.hoodAngle, kShots.AUTO_SIDE.shooterVelocity),
                    new SequentialCommandGroup(
                        new InstantCommand(intake::stopIntake, intake),
                        new WaitCommand(0.5),
                        new InstantCommand(indexer::setIdle, indexer)
                    )
                ),
                new ParallelCommandGroup(
                    getCommand(fiveBallPaths[1], true),
                    new InstantCommand(intake::runIntake, intake),
                    new InstantCommand(indexer::setIntake, indexer)
                ),
                new WaitCommand(2),
                new ParallelCommandGroup(
                    getCommand(fiveBallPaths[2], true)
                ),
                new ParallelCommandGroup(
                    new Shoot(60000, 2700),
                    new SequentialCommandGroup(
                        new InstantCommand(intake::stopIntake, intake),
                        new WaitCommand(0.5),
                        new InstantCommand(indexer::setIdle, indexer)
                    )
                ),
                new ParallelCommandGroup(
                    getCommand(fiveBallPaths[3], true),
                    new InstantCommand(intake::runIntake, intake),
                    new InstantCommand(indexer::setIntake, indexer)
                ),
                new ParallelCommandGroup(
                    new Shoot(kShots.AUTO_TARMAC.hoodAngle, kShots.AUTO_TARMAC.shooterVelocity),
                    new SequentialCommandGroup(
                        new InstantCommand(intake::stopIntake, intake),
                        new WaitCommand(0.5),
                        new InstantCommand(indexer::setIdle, indexer)
                    )
                )
            )
        );
    }

    private Command getCommand(String pathName, boolean isFirstPath) {
        PathPlannerTrajectory path = PathPlanner.loadPath(pathName, kSwerve.MAX_ANGULAR_VELOCITY, kSwerve.MAX_ACCELERATION);

        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                if (isFirstPath)
                    swerve.resetOdometry(getInitialPose(path));
            }, swerve),
            new PPSwerveControllerCommand(
                path, 
                swerve::getPose, 
                kSwerve.SWERVE_KINEMATICS, 
                kSwerve.X_CONTROLLER, 
                kSwerve.Y_CONTROLLER, 
                kSwerve.ANGLE_CONTROLLER, 
                (s -> swerve.setModuleStates(s)), 
                swerve
            )
        );
    }

    private Pose2d getInitialPose(PathPlannerTrajectory path) {
        return new Pose2d(path.getInitialPose().getTranslation(), path.getInitialState().holonomicRotation);
    }
}

