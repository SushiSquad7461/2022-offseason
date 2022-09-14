package frc.robot;

import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import SushiFrcLib.Motor.SDSSwerveModuleConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public final class Constants {
        public final static class kSwerveDrive {
                public static final Mk4SwerveModuleHelper.GearRatio GEAR_RATIO = Mk4SwerveModuleHelper.GearRatio.L2;
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

        public static class Ports {
        }

        public static void setup() {
        }
}
