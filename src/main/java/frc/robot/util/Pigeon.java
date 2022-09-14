package frc.robot.util;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import SushiFrcLib.Math.Conversion;
import frc.robot.Constants;

public class Pigeon extends Gyro {
    private final WPI_Pigeon2 gyro;
    private static Pigeon instance;

    public static Pigeon getInstance() {
        if (instance == null) {
            instance = new Pigeon();
        }
        return instance;
    }

    private Pigeon() {
        gyro = new WPI_Pigeon2(Constants.Ports.PIGEON);
    }

    @Override
    public void zero() {
        gyro.setYaw(0);
    }

    @Override
    public double getAngle() {
        return Conversion.normalizeAngle(gyro.getYaw());
    }
}
