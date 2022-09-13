package frc.robot.util;

import com.ctre.phoenix.sensors.Pigeon2;

import frc.robot.Constants;

public class Pigeon extends Gyro {
    private final Pigeon2 gyro;
    private static Pigeon instance;

    public static Pigeon getInstance() {
        if (instance == null) {
            instance = new Pigeon();
        }
        return instance;
    }

    private Pigeon() {
        gyro = new Pigeon2(Constants.Ports.PIGEON);
    }

    @Override
    public double getAngle() {
        return gyro.getAbsoluteCompassHeading();
    }


    
}
