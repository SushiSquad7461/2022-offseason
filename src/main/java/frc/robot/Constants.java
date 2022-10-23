package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import SushiFrcLib.CheesyLibUtil.InterpolatingDouble;
import SushiFrcLib.CheesyLibUtil.InterpolatingTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.util.SwerveModuleConstants;

public final class Constants {
    public static final double STICK_DEADBAND = 0.1;
    public static final boolean TUNING_MODE = false;

    public final static class kIntake {
        public static final double INTAKE_SPEED = 0.5;
        public static final double HOPPER_SPEED = 0.3;

        public static final TalonFXInvertType INTAKE_INVERSION = TalonFXInvertType.Clockwise;
        public static final TalonFXInvertType HOPPER_INVERSION = TalonFXInvertType.Clockwise;

        public static final int CURRENT_LIMIT = 20; //30
    }

    public static class kHood {
        public static final int CURRENT_LIMIT = 25; //35
        public static final TalonFXInvertType INVERSION = TalonFXInvertType.Clockwise;
        public static final NeutralMode NEUTRAL_MODE = NeutralMode.Brake;
        public static final double kP = 0.7;
        public static final double kI = 0;
        public static final double kD = 0.1;
        public static final double kF = 0;
        public static final double MAX_POS = 150000;
        public static final double HOOD_ERROR = 200;
        public static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> POS_MAP = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>();
        public static final double OFFSET = 0;

        public static final double TENSION_CURRENT = 0.2;
        public static final double TENSION_SPEED = -0.1;

        static {
            // First val is disntance from goal (feet), Second val is hood pos
            // in encoder tiks
            POS_MAP.put(new InterpolatingDouble(0.0), new InterpolatingDouble(20000.0));
            POS_MAP.put(new InterpolatingDouble(4.0), new InterpolatingDouble(40000.0));
            POS_MAP.put(new InterpolatingDouble(7.0), new InterpolatingDouble(50000.0));
            POS_MAP.put(new InterpolatingDouble(10.0), new InterpolatingDouble(80000.0));
            POS_MAP.put(new InterpolatingDouble(13.0), new InterpolatingDouble(90000.0));
        }
    }

    public static class kShooter {
        public static final double SETPOINT_RPM = 0;
        public static final double ERROR_TOLERANCE = 50; // 60
        public static final int CURRENT_LIMIT = 40;
        public static final double kP = 0.270000; // 0.07
        public static final double kI = 0.0;
        public static final double kD = 3.0; // 0
        public static final double kF = 0.0527;
        public static final double OFFSET = 0;

        public static final double TX_OFFSET = 0;
        public static final double PID_TOLERANCE_DEGREES = 2;
        public static final double PID_SPEED_TOLERANCE_DEGREES_PER_SECOND = 720; // TODO: pick actual values lol
        public static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> POS_MAP = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>();

        static {
            // First val is disntance from goal in feet, Second val is shooterRPM
            POS_MAP.put(new InterpolatingDouble(0.0), new InterpolatingDouble(2300.0));
            POS_MAP.put(new InterpolatingDouble(4.0), new InterpolatingDouble(2300.0));
            POS_MAP.put(new InterpolatingDouble(7.0), new InterpolatingDouble(2550.0));
            POS_MAP.put(new InterpolatingDouble(10.0), new InterpolatingDouble(2700.0));
            POS_MAP.put(new InterpolatingDouble(13.0), new InterpolatingDouble(3100.0));
        }
    }

    public static class kIndexer {
        public static final double EJECT_DELAY = 0.1; // in seconds

        public static final double EJECTER_SPEED = 1;
        public static final double KICKER_SPEED = 1;
        public static final double FEADER_SPEED = 0.8;
    }

    public static class kPorts {
        public static final String CANIVORE_NAME = "Sussy Squad";
        public static final int PIGEON_ID = 13;

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

    public static final class kSwerve {
        public static final boolean OPEN_LOOP = false;
        public static final boolean FEILD_RELATIVE = false;

        public static final boolean GYRO_INVERSION = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(21.73);
        public static final double WHEEL_BASE = Units.inchesToMeters(21.73);
        public static final double WHEEL_DIAMATER = Units.inchesToMeters(4);
        public static final double WHEEL_CIRCUMFRANCE = WHEEL_DIAMATER * Math.PI;

        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        public static final double DRIVE_GEAR_RATIO = 6.75; // 6.86:1
        public static final double ANGLE_GEAR_RATIO = (150.0 / 7.0); // 12.8:1

        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

        /* Swerve Current Limiting */
        public static final int ANGLE_CONTINUSE_CURRENT_LIMIT = 20; //25
        public static final int ANGLE_PEAK_CURRENT_LIMIT = 30; //40
        public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

        public static final int DRIVE_CONTINUSE_CURRENT_LIMIT = 30; //35
        public static final int DRIVE_PEAK_CURRENT_LIMIT = 50; //60
        public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        /* Angle Motor PID Values */
        public static final double ANGLE_P = 0.8;
        public static final double ANGLE_I = 0.0;
        public static final double ANGLE_D = 12.0;
        public static final double ANGLE_F = 0.0;

        /* Drive Motor PID Values */
        public static final double DRIVE_P = 0.18;
        public static final double DRIVE_I = 0.0;
        public static final double DRIVE_D = 0.0;
        public static final double DRIVE_F = 0.05;

        /* Swerve Profiling Values */
        public static final double MAX_ACCELERATION = 2; // 2
        public static final double MAX_SPEED = 11.5; // 4.5 meters per second
        public static final double MAX_ANGULAR_VELOCITY = 20; // 11.5

        public static final PIDController X_CONTROLLER = new PIDController(2, 0, 0);
        public static final PIDController Y_CONTROLLER = new PIDController(2, 0, 0);
        public static final ProfiledPIDController ANGLE_CONTROLLER = new ProfiledPIDController(4, 0, 0,
                new Constraints(MAX_ANGULAR_VELOCITY, 17));

        /* Neutral Modes */
        public static final NeutralMode ANGLE_NEUTRAL_MODE = NeutralMode.Coast;
        public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;

        /* Motor Inverts */
        public static final boolean DRIVE_INVERSION = false;
        public static final boolean ANGLE_INVERSION = true; //make false if we have a stroke

        /* Angle Encoder Invert */
        public static final boolean CANCODER_INVERSION = true;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int DRIVE_MOTOR_ID = 1;
            public static final int ANGLE_MOTOR_ID = 3;
            public static final int CAN_CODER_ID = 2;
            public static final double ANGLE_OFFSET = 89.648438;
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int DRIVE_MOTOR_ID = 10;
            public static final int ANGLE_MOTOR_ID = 12;
            public static final int CAN_CODER_ID = 11;
            public static final double ANGLE_OFFSET = 195.380859;
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int DRIVE_MOTOR_ID = 4;
            public static final int ANGLE_MOTOR_ID = 6;
            public static final int CAN_CODER_ID = 5;
            public static final double ANGLE_OFFSET = 67.675781;
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int DRIVE_MOTOR_ID = 7;
            public static final int ANGLE_MOTOR_ID = 9;
            public static final int CAN_CODER_ID = 8;
            public static final double ANGLE_OFFSET = 69.785156;
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

    }

    public static final class kVision {
        public static final double LIME_LIGHT_HEIGHT = 2.75; // Limelight height in feet
        public static final double HUB_HEIGHT = 8.666667; // hub height in feet
        public static final double LIME_LIGHT_TO_HUB_HEIGHT = HUB_HEIGHT - LIME_LIGHT_HEIGHT; // height difference
                                                                                              // between
        // the hub and limelight
        public static final double LIME_LIGHT_MOUNT_ANGLE = 39.5; // Limelight angle in degrees from the horizon
    }

    public static final class kOI {
        public static final int DRIVE_TRANSLATION_Y = XboxController.Axis.kLeftY.value;
        public static final int DRIVE_TRANSLATION_X = XboxController.Axis.kLeftX.value;
        public static final int DRIVE_ROTATE = XboxController.Axis.kRightX.value;

        public static final int RUN_INTAKE = XboxController.Button.kLeftBumper.value;
        public static final int REVERSE_INTAKE = XboxController.Button.kX.value;

        public static final int BACK_INDEXER = XboxController.Button.kY.value;

        public static final int ZERO_GYRO = XboxController.Button.kA.value;
        public static final int ZERO_SHOOTER_HOOD = XboxController.Button.kA.value;

        public static final int AUTO_SHOOT = XboxController.Button.kRightBumper.value;
        public static final int FENDER_SHOOT = XboxController.Button.kB.value;

        public static final int UPDATE_ENCODER = XboxController.Button.kY.value;
    }

    enum kShots {
        FENDER(0, 2300),
        AUTO_TARMAC(70000, 2600),
        AUTO_SIDE(70000, 2550);

        public double hoodAngle;
        public double shooterVelocity;

        private kShots(double hoodAngle, double shooterVelocity) {
            this.hoodAngle = hoodAngle;
            this.shooterVelocity = shooterVelocity;
        }
    }
}
