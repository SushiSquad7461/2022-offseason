package frc.robot;

import SushiFrcLib.Constants.SushiConstants;
import edu.wpi.first.wpilibj.XboxController;

public class OI {
    public static final XboxController driver = new XboxController(SushiConstants.OI.DRIVER_PORT);

    public static double getLeftStickX() {
        return driver.getLeftX();
    }

    public static double getLeftStickY() {
        return driver.getLeftY();
    }
    
    public static double getRightStickX() {
        return driver.getRightX();
    }
}