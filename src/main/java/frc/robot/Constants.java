package frc.robot;

import org.ejml.simple.SimpleMatrix;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import SushiFrcLib.Motor.MotorConfig;
import SushiFrcLib.Motor.SwerveModuleConfig;
import SushiFrcLib.State.State.MotorDirection;
import SushiFrcLib.State.State.MotorNeturalState;
import SushiFrcLib.State.State.MotorType;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public final class Constants{
        public static final double stickDeadband = 0.1;

        public static final class Swerve {
            public static final int pigeonID = 13;
            public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-
    
            /* Drivetrain Constants */
            public static final double trackWidth = Units.inchesToMeters(21.73);
            public static final double wheelBase = Units.inchesToMeters(21.73);
            public static final double wheelDiameter = Units.inchesToMeters(4);
            public static final double wheelCircumference = wheelDiameter * Math.PI;
    
            public static final double openLoopRamp = 0.25;
            public static final double closedLoopRamp = 0.0;
    
            public static final double driveGearRatio = 6.75; //6.86:1
            public static final double angleGearRatio = (150.0 / 7.0); //12.8:1
    
            public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                    new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                    new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                    new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0),
                    new Translation2d(-wheelBase / 2.0, trackWidth / 2.0));

            public static final Matrix<N3, N1> kOdomStateStdDevs = 
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.0, 0.0, 0.0);

            public static final Matrix<N1, N1> kOdomLocalMeasurementStdDevs = 
                new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.0);

            public static final Matrix<N3, N1> visionMeasurementStdDevs = 
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.0, 0.0, 0.0);

            public static final double kHubPosXMeters = 0.0;
            public static final double kHubPosYMeters = 0.0;
    
            /* Swerve Current Limiting */
            public static final int angleContinuousCurrentLimit = 25;
            public static final int anglePeakCurrentLimit = 40;
            public static final double anglePeakCurrentDuration = 0.1;
            public static final boolean angleEnableCurrentLimit = true;
    
            public static final int driveContinuousCurrentLimit = 35;
            public static final int drivePeakCurrentLimit = 60;
            public static final double drivePeakCurrentDuration = 0.1;
            public static final boolean driveEnableCurrentLimit = true;
    
            /* Angle Motor PID Values */
            public static final double angleKP = 0.6;
            public static final double angleKI = 0.0;
            public static final double angleKD = 12.0;
            public static final double angleKF = 0.0;
    
            /* Drive Motor PID Values */
            public static final double driveKP = 0.10;
            public static final double driveKI = 0.0;
            public static final double driveKD = 0.0;
            public static final double driveKF = 0.0;
    
            /* Drive Motor Characterization Values */
            public static final double driveKS = (0.667 / 12); //divide by 12 to convert from volts to percent output for CTRE
            public static final double driveKV = (2.44 / 12);
            public static final double driveKA = (0.27 / 12);
    
            /* Swerve Profiling Values */
            public static final double maxSpeed = 4.5; //meters per second
            public static final double maxAngularVelocity = 11.5;
    
            /* Neutral Modes */
            public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
            public static final NeutralMode driveNeutralMode = NeutralMode.Brake;
    
            /* Motor Inverts */
            public static final boolean driveMotorInvert = false;
            public static final boolean angleMotorInvert = false;
    
            /* Angle Encoder Invert */
            public static final boolean canCoderInvert = true;
    
            /* Module Specific Constants */
            /* Front Left Module - Module 0 */
            public static final class Mod0 {
                public static final int driveMotorID = 1;
                public static final int angleMotorID = 3;
                public static final int canCoderID = 2;
                public static final double angleOffset = 90.175781;
                public static final SwerveModuleConstants constants = 
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
            }
    
            /* Front Right Module - Module 1 */
            public static final class Mod1 {
                public static final int driveMotorID = 10;
                public static final int angleMotorID = 12;
                public static final int canCoderID = 11;
                public static final double angleOffset = 194.853516;
                public static final SwerveModuleConstants constants = 
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
            }
            
            /* Back Left Module - Module 2 */
            public static final class Mod2 {
                public static final int driveMotorID = 4;
                public static final int angleMotorID = 6;
                public static final int canCoderID = 5;
                public static final double angleOffset = 66.533203;
                public static final SwerveModuleConstants constants = 
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
            }
    
            /* Back Right Module - Module 3 */
            public static final class Mod3 {
                public static final int driveMotorID = 7;
                public static final int angleMotorID = 9;
                public static final int canCoderID = 8;
                public static final double angleOffset = 75.498047;
                public static final SwerveModuleConstants constants = 
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
            }
    
        }
    
        public static final class AutoConstants {
            public static final double kMaxSpeedMetersPerSecond = 3;
            public static final double kMaxAccelerationMetersPerSecondSquared = 3;
            public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
            public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
        
            public static final double kPXController = 1;
            public static final double kPYController = 1;
            public static final double kPThetaController = 1;
        
            // Constraint for the motion profilied robot angle controller
            public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                    kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
        }

        public static final class Vision {
            public static final double kLimeLightMountAngleDegrees = 0.0;
            public static final double kLimeLightLensHeightMeters = 0.0;
            public static final double kHubTargetHeightMeters = 0.0;
        }    
}
