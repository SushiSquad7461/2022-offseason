package frc.robot.util;

import com.kauailabs.navx.frc.AHRS;

import SushiFrcLib.Math.Conversion;
import edu.wpi.first.wpilibj.SPI;

public class NavX extends Gyro {
   private final AHRS gyro;
   private double offset = 0;
   private static NavX instance;

   public static NavX getInstance() {
       if (instance == null) {
           instance = new NavX();
       }
       return instance;
   }
   
   public NavX() {
       gyro = new AHRS(SPI.Port.kMXP);
   }

   public boolean isCalibrated() {
       return !gyro.isCalibrating();
   }

   public void setOffset(double newOffset) {
        offset = newOffset;
   }

   @Override
   public void zero() {
       setOffset(gyro.getAngle());
   }

   public double getAngle() {
       return Conversion.normalizeAngle(gyro.getAngle() - offset);
   }
}
