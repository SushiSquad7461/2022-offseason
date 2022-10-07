package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import SushiFrcLib.Motor.MotorConfig;
import SushiFrcLib.Motor.SwerveModuleConfig;
import SushiFrcLib.State.State.MotorDirection;
import SushiFrcLib.State.State.MotorNeturalState;
import SushiFrcLib.State.State.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import SushiFrcLib.CheesyLibUtil.InterpolatingDouble;
import SushiFrcLib.CheesyLibUtil.InterpolatingTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final double stickDeadband = 0.1;
    public static final boolean TUNING_MODE = true;

    public final static class kIntake {
        public static final double INTAKE_SPEED = 0.5;
        public static final double HOPPER_SPEED = 0.3;

        public static final TalonFXInvertType INTAKE_INVERSION = TalonFXInvertType.Clockwise;
        public static final TalonFXInvertType HOPPER_INVERSION = TalonFXInvertType.Clockwise;

        public static final int CURRENT_LIMIT = 30;
    }

    public static class kHood {
        public static final int CURRENT_LIMIT = 35;
        public static final TalonFXInvertType INVERSION = TalonFXInvertType.Clockwise;
        public static final NeutralMode NEUTRAL_MODE = NeutralMode.Brake;
        public static final double kP = 0.8;
        public static final double kI = 0;
        public static final double kD = 0.1;
        public static final double kF = 0;
        public static final double maxPos = 150000;
        public static final double kHoodError = 500;
        public static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> posMap = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>();

        static {
            // First val is disntance from goal (TODO: INSERT UNITS), Second val is hood pos
            // in encoder tiks
            posMap.put(new InterpolatingDouble(0.0), new InterpolatingDouble(0.0));
            posMap.put(new InterpolatingDouble(3.0), new InterpolatingDouble(40000.0));
            posMap.put(new InterpolatingDouble(5.0), new InterpolatingDouble(50000.0));
            posMap.put(new InterpolatingDouble(7.0), new InterpolatingDouble(60000.0));
            posMap.put(new InterpolatingDouble(9.0), new InterpolatingDouble(70000.0));
            posMap.put(new InterpolatingDouble(11.0), new InterpolatingDouble(80000.0));
            posMap.put(new InterpolatingDouble(13.6), new InterpolatingDouble(90000.0));
        }
    }

    public static class kShooter {
        public static final double SETPOINT_RPM = 0;
        public static final double ERROR_TOLERANCE = 60;
        public static final int CURRENT_LIMIT = 40;
        public static final double kP = 0.2;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kF = 0.060500;

        public static final double TX_OFFSET = 0;
        public static final double PID_TOLERANCE_DEGREES = 0.5;
        public static final double PID_SPEED_TOLERANCE_DEGREES_PER_SECOND = 720; // TODO: pick actual values lol
        public static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> posMap = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>();

        static {
            // First val is disntance from goal in feet, Second val is shooterRPM
            posMap.put(new InterpolatingDouble(0.0), new InterpolatingDouble(0.0));
            posMap.put(new InterpolatingDouble(3.0), new InterpolatingDouble(2250.0));
            posMap.put(new InterpolatingDouble(5.0), new InterpolatingDouble(2300.0));
            posMap.put(new InterpolatingDouble(7.0), new InterpolatingDouble(2400.0));
            posMap.put(new InterpolatingDouble(9.0), new InterpolatingDouble(2600.0));
            posMap.put(new InterpolatingDouble(11.0), new InterpolatingDouble(2800.0));
            posMap.put(new InterpolatingDouble(13.6), new InterpolatingDouble(3100.0));
        }
    }

    public static class kIndexer {
        public static final double colorSensorThreasholdRed = 0.33;
        public static final double colorSensorThreasholdBlue = 0.27;

        public static final double ejectDelaySeconds = 0.1;

        public static final double EJECTER_SPEED = 1;
        public static final double KICKER_SPEED = 1;
        public static final double FEADER_SPEED = 0.8;
    }

    public static class Ports {
        public static final int BOTTOM_BEAM_BREAK = 0;
        public static final int UPPER_BEAM_BREAK = 1;

        public static final int SHOOTER_LEFT_MOTOR = 45;
        public static final int SHOOTER_RIGHT_MOTOR = 46;
        public static final int KICKER_MOTOR = 43;
        public static final int EJECTER_MOTOR = 44;
        public static final int FEEDER_MOTOR = 41;
        public static final int HOOD_MOTOR = 47;

        public static final int INTAKE_MOTOR = 49;
        public static final int HOPPER_MOTOR = 48;

    }

    public static final class Swerve {
        public static final int pigeonID = 13;
        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.73);
        public static final double wheelBase = Units.inchesToMeters(21.73);
        public static final double wheelDiameter = Units.inchesToMeters(4);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = 6.75; // 6.86:1
        public static final double angleGearRatio = (150.0 / 7.0); // 12.8:1

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0));

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
        public static final double angleKP = 0.8; // 0.6 before
        public static final double angleKI = 0.0;
        public static final double angleKD = 12.0;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.18;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.05;

        /* Swerve Profiling Values */
        public static final double maxAcceleration = 0.2;
        public static final double maxSpeed = 2.5; // 4.5 meters per second
        public static final double maxAngularVelocity = 8.5; // 11.5

        public static final PIDController xController = new PIDController(0.05, 0, 0);
        public static final PIDController yController = new PIDController(0.05, 0, 0);
        public static final ProfiledPIDController angleController = new ProfiledPIDController(0.05, 0, 0,
                new Constraints(Constants.Swerve.maxAngularVelocity, 2.0));

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
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                    angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 12;
            public static final int canCoderID = 11;
            public static final double angleOffset = 194.853516;
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                    angleMotorID, canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 5;
            public static final double angleOffset = 66.533203;
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                    angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 9;
            public static final int canCoderID = 8;
            public static final double angleOffset = 70.400391;
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                    angleMotorID, canCoderID, angleOffset);
        }

    }

    public static final class VisionConstants {
        public static final double kLimeLightHeight = 2.75; // Limelight height in feet
        public static final double kHubHeight = 8.666667; // hub height in feet
        public static final double kLimeLightToHubHeight = kHubHeight - kLimeLightHeight; // height difference between
                                                                                          // the hub and limelight
        public static final double kLimeLightMountAngle = 39.5; // Limelight angle in degrees from the horizon
        public static final double kLimeLightVerticalFOV = 49.7; // Limelight vertical field of view in degrees
        public static final double kLimeLightHorizontalFOV = 59.6; // Limelight horizontal field of view in degrees
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
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

}
