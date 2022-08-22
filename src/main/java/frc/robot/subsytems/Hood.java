package frc.robot.subsytems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import SushiFrcLib.ChesyLibUtil.InterpolatingDouble;
import SushiFrcLib.Motor.MotorHelper;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import frc.robot.Constants.kHood;

public class Hood extends SubsystemBase {
    WPI_TalonFX motor;
    double pos = 0;

    Hood() {
        motor = MotorHelper.createFalconMotor(Ports.HOOD_MOTOR, kHood.CURRENT_LIMIT, kHood.INVERSION, kHood.NEUTRAL_MODE, kHood.kP, kHood.kI, kHood.kD, kHood.kF);
        motor.setSelectedSensorPosition(0);
    }

    void setPos(double newPos) {
        if (0 <= newPos && newPos <= kHood.maxPos) {
            pos = newPos;
        }
    }

    void setPosBasedOnDistance(double distance) {
        pos = kHood.posMap.getInterpolated(new InterpolatingDouble(distance)).value;
    }

    @Override
    public void periodic() {
       motor.set(ControlMode.Position, pos);
    } 
}