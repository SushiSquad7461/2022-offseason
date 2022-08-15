package frc.robot;

import SushiFrcLib.Motor.MotorConfig;
import SushiFrcLib.Motor.SwerveModuleConfig;
import SushiFrcLib.State.State.MotorDirection;
import SushiFrcLib.State.State.MotorNeturalState;
import SushiFrcLib.State.State.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public final class Constants{
        public static final class kIntake {
                public static final int PIVOT_CURRENT_LIMIT = 30;
                public static final int ROLLER_CURRENT_LIMIT = 30;
                public static final int EXTENDED_SETPOINT = 200;
                public static final double kPPivot = 0;
                public static final double kIPivot = 0;
                public static final double kDPivot = 0;
                public static final double kFPivot = 0;
        }

        public static class Ports {
                public static final int INTAKE_PIVOT = -1;
                public static final int INTAKE_ROLLER = -1;
        }

        public static class kSwerve {
                public static final double WHEEL_BASE = 0;
                public static final double MAX_SPEED = 0;
                public static final double MAX_ACCELERATION = 0;
                public static final SwerveModuleConfig[] CONFIGS = new SwerveModuleConfig[]{
                        new SwerveModuleConfig(
                                new MotorConfig(0, 0, MotorNeturalState.BRAKE, MotorDirection.CLOCKWISE, 0, 0, 0, 0, MotorType.BRUSHLESS), 
                                new MotorConfig(0, 0, MotorNeturalState.BRAKE, MotorDirection.CLOCKWISE, 0, 0, 0, 0, MotorType.BRUSHLESS), 0, 0),
                        new SwerveModuleConfig(
                                new MotorConfig(0, 0, MotorNeturalState.BRAKE, MotorDirection.CLOCKWISE, 0, 0, 0, 0, MotorType.BRUSHLESS), 
                                new MotorConfig(0, 0, MotorNeturalState.BRAKE, MotorDirection.CLOCKWISE, 0, 0, 0, 0, MotorType.BRUSHLESS), 0, 0),
                        new SwerveModuleConfig(
                                new MotorConfig(0, 0, MotorNeturalState.BRAKE, MotorDirection.CLOCKWISE, 0, 0, 0, 0, MotorType.BRUSHLESS), 
                                new MotorConfig(0, 0, MotorNeturalState.BRAKE, MotorDirection.CLOCKWISE, 0, 0, 0, 0, MotorType.BRUSHLESS), 0, 0),
                        new SwerveModuleConfig(
                                new MotorConfig(0, 0, MotorNeturalState.BRAKE, MotorDirection.CLOCKWISE, 0, 0, 0, 0, MotorType.BRUSHLESS), 
                                new MotorConfig(0, 0, MotorNeturalState.BRAKE, MotorDirection.CLOCKWISE, 0, 0, 0, 0, MotorType.BRUSHLESS), 0, 0)};

                public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                        new Translation2d(WHEEL_BASE/2.0, WHEEL_BASE/2.0), 
                        new Translation2d(-WHEEL_BASE/2.0, WHEEL_BASE/2.0), 
                        new Translation2d(-WHEEL_BASE/2.0, -WHEEL_BASE/2.0), 
                        new Translation2d(WHEEL_BASE/2.0, -WHEEL_BASE/2.0));
                
                public static final PIDController X_CONTROLLER = new PIDController(0, 0, 0);
                public static final PIDController Y_CONTROLLER = new PIDController(0, 0, 0);
                public static final ProfiledPIDController ANGLE_CONTROLLER = new ProfiledPIDController(0, 0, 0, new Constraints(0, 0));
        }

        public static void setup() {
        }
}
