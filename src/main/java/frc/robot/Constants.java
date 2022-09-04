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
        public static double convertRPMToTrans(double rpm) {
                return rpm * 2048.0 / 600.0;
        }

        public static double convertTransToRPM(double trans) {
                return trans * 600.0 / 2048.0;
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
        }

        public static class Ports {
                public static final int SHOOTER_LEFT_MOTOR = 5;
                public static final int SHOOTER_RIGHT_MOTOR = 14;
                public static final int KICKER_MOTOR = 43;
                public static final int EJECTER_MOTOR = 44;
                public static final int FEEDER_MOTOR = 41;
        }

        public static void setup() {
        }
}
