package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

public final class Constants{
        public static final class Ports {
                public static final int RIGHT_CLIMB_MOTOR = 0;
                public static final int LEFT_CLIMB_MOTOR = 0;
        }

        public static final class kClimb {

            public static final CANSparkMaxLowLevel.MotorType MOTOR_TYPE = CANSparkMaxLowLevel.MotorType.kBrushless;
            public static final boolean LEFT_INVERSION = false;
            public static final boolean RIGHT_INVERSION = !LEFT_INVERSION;
            public static final int CURRENT_LIMIT = 30;
            public static final IdleMode IDLE_MODE = IdleMode.kBrake;

            public static final double OPEN_LOOP_SPEED = 0.3;

            public static final double kP = 0;
            public static final double kI = 0;
            public static final double kD = 0;

            public static final double kS = 0;
            public static final double kV = 0;
            public static final double kA = 0;
        }

        public static void setup() {
        }
}
