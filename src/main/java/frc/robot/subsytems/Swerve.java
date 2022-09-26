package frc.robot.subsytems;


import com.ctre.phoenix.sensors.PigeonIMU;

import SushiFrcLib.Math.Vector2;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimeLight;

public class Swerve extends SubsystemBase {
    public SwerveDrivePoseEstimator swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public PigeonIMU gyro;

    public Swerve() {
        gyro = new PigeonIMU(Constants.Swerve.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();
        
        swerveOdometry = new SwerveDrivePoseEstimator(
            getYaw(),
            new Pose2d(), // This should probably change in the future
            Constants.Swerve.swerveKinematics, 
            Constants.Swerve.kOdomStateStdDevs,
            Constants.Swerve.kOdomLocalMeasurementStdDevs,
            Constants.Swerve.visionMeasurementStdDevs
        );

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(fieldRelative 
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), 
                    translation.getY(), 
                    rotation, 
                    getYaw()
                )
                : new ChassisSpeeds(
                    translation.getX(), 
                    translation.getY(), 
                    rotation
                )
            );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(pose, getYaw());
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getYaw(), getStates());  

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }

        if (!LimeLight.getInstance().canSeeTarget()) return;

        double limeLightDistance = LimeLight.getInstance().getDistance();
        double limeLightHeading = LimeLight.getInstance().getHeading();
        

        Pose2d robotPos = getPose();
        Vector2 robotRelativeToHub = new Vector2(
            robotPos.getX() + Constants.Swerve.kHubPosXMeters,
            robotPos.getY() + Constants.Swerve.kHubPosYMeters
        );

        // Since we don't care where around the hub we are, let's just rotate
        // the limelight pos to be in the same direction as robot pos
        Vector2 limeLightRelativeToHub = robotRelativeToHub.normal()
            .scale(limeLightDistance);

        Vector2 robotForward = new Vector2(
            robotPos.getRotation().getSin(),
            robotPos.getRotation().getCos() // Y+ is always forward
        );

        // Since getAngleBetween will only return the smallest angle, we can't use it
        // Reference: https://stackoverflow.com/a/2150475
        double limeLightOdomAngleToHubDiff = Math.atan2(
            robotForward.cross(robotRelativeToHub),
            robotForward.dot(robotRelativeToHub)
        ) - limeLightHeading;

        swerveOdometry.addVisionMeasurement(
            new Pose2d(
                limeLightRelativeToHub.x - Constants.Swerve.kHubPosXMeters,
                limeLightRelativeToHub.y - Constants.Swerve.kHubPosYMeters,
                robotPos.getRotation().rotateBy(Rotation2d.fromDegrees(limeLightOdomAngleToHubDiff))
            ), 
            Timer.getFPGATimestamp()
        );
    }
}