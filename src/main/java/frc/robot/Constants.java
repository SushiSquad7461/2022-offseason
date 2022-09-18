package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import SushiFrcLib.CheesyLibUtil.InterpolatingDouble;
import SushiFrcLib.CheesyLibUtil.InterpolatingTreeMap;
import SushiFrcLib.Motor.SDSSwerveModuleConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public final class Constants {
        public final static class kSwerveDrive {
                public static final Mk4iSwerveModuleHelper.GearRatio FOUR_I_GEAR_RATIO = Mk4iSwerveModuleHelper.GearRatio.L2;

                public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.52705;
                public static final double DRIVETRAIN_WHEELBASE_METERS = 0.52705;

                public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
                    SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
                    SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;
                public static final double MAX_ACCELERATION = 2.0;
                public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);
                public static final double MAX_VOLTAGE = 12;
        
                public static final SDSSwerveModuleConfig FRONT_LEFT = new SDSSwerveModuleConfig(1, 3, 2, -Math.toRadians(268.021537162), "Front Left Module", 2); // module 1
                public static final SDSSwerveModuleConfig FRONT_RIGHT = new SDSSwerveModuleConfig(10, 12, 11, -Math.toRadians(164.53125), "Front Right Module", 4); // module 4
                public static final SDSSwerveModuleConfig BACK_LEFT = new SDSSwerveModuleConfig(4, 6, 5, -Math.toRadians(114.43359375), "Back Left Module", 6); // module 2
                public static final SDSSwerveModuleConfig BACK_RIGHT = new SDSSwerveModuleConfig(7, 9, 8, -Math.toRadians(110.21484375), "Back Right Module", 8); // module 3
        
                public static final PIDController X_CONTROLLER = new PIDController(0.001, 0, 0);
                public static final PIDController Y_CONTROLLER = new PIDController(0.001, 0, 0);
                public static final ProfiledPIDController ANGLE_CONTROLLER = new ProfiledPIDController(0.001, 0, 0,
                        new Constraints(kSwerveDrive.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 2.0));
        }
        public static class kHood {
                public static final int CURRENT_LIMIT = 35;
                public static final TalonFXInvertType INVERSION = TalonFXInvertType.Clockwise;
                public static final NeutralMode NEUTRAL_MODE = NeutralMode.Brake;
                public static final double kP = 0.8;
                public static final double kI = 0;
                public static final double kD = 0.1;
                public static final double kF = 0;
                public static final double maxPos = 180;
                public static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> posMap = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>();

                static {
                        // First val is disntance from goal (TODO: INSERT UNITS), Second val is hood pos
                        // in encoder tiks
                        posMap.put(new InterpolatingDouble(0.0), new InterpolatingDouble(0.0));
                }  
        }
        public static class kShooter {
                public static final double SETPOINT_RPM = 0;
                public static final double ERROR_TOLERANCE = 10;
                public static final int CURRENT_LIMIT = 40;
                public static final double kP = 0.0;
                public static final double kI = 0.0;
                public static final double kD = 0.0;
                public static final double kF = 0.0;
        }

        public static class kIndexer {
                public static final double colorSensorThreasholdRed = 0.33;
                public static final double colorSensorThreasholdBlue = 0.27;

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
        }

        public static void setup() {
        }

        public static final boolean TUNING_MODE = true;

}