package frc.robot;

import SushiFrcLib.Motor.MotorConfig;
import SushiFrcLib.Motor.SwerveModuleConfig;
import SushiFrcLib.State.State.MotorDirection;
import SushiFrcLib.State.State.MotorNeturalState;
import SushiFrcLib.State.State.MotorType;

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
                public static final SwerveModuleConfig frontLeft = new SwerveModuleConfig(new MotorConfig(0, 0, MotorNeturalState.BRAKE, MotorDirection.CLOCKWISE, 0, 0, 0, 0, MotorType.BRUSHLESS), new MotorConfig(0, 0, MotorNeturalState.BRAKE, MotorDirection.CLOCKWISE, 0, 0, 0, 0, MotorType.BRUSHLESS), 0, 0);
                public static final SwerveModuleConfig frontRight = new SwerveModuleConfig(new MotorConfig(0, 0, MotorNeturalState.BRAKE, MotorDirection.CLOCKWISE, 0, 0, 0, 0, MotorType.BRUSHLESS), new MotorConfig(0, 0, MotorNeturalState.BRAKE, MotorDirection.CLOCKWISE, 0, 0, 0, 0, MotorType.BRUSHLESS), 0, 0);
                public static final SwerveModuleConfig backLeft = new SwerveModuleConfig(new MotorConfig(0, 0, MotorNeturalState.BRAKE, MotorDirection.CLOCKWISE, 0, 0, 0, 0, MotorType.BRUSHLESS), new MotorConfig(0, 0, MotorNeturalState.BRAKE, MotorDirection.CLOCKWISE, 0, 0, 0, 0, MotorType.BRUSHLESS), 0, 0);
                public static final SwerveModuleConfig backRight = new SwerveModuleConfig(new MotorConfig(0, 0, MotorNeturalState.BRAKE, MotorDirection.CLOCKWISE, 0, 0, 0, 0, MotorType.BRUSHLESS), new MotorConfig(0, 0, MotorNeturalState.BRAKE, MotorDirection.CLOCKWISE, 0, 0, 0, 0, MotorType.BRUSHLESS), 0, 0);
        }

        public static void setup() {
        }
}
