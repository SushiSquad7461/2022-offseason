package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import SushiFrcLib.CheesyLibUtil.InterpolatingDouble;
import SushiFrcLib.CheesyLibUtil.InterpolatingTreeMap;

public final class Constants {
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

        public static class Ports {
                public static final int SHOOTER_LEFT_MOTOR = 45;
                public static final int SHOOTER_RIGHT_MOTOR = 46;
                public static final int HOOD_MOTOR = 47;
        }

        public static void setup() {
        }

        public static final boolean TUNING_MODE = true;

}