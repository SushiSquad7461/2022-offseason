package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import SushiFrcLib.CheesyLibUtil.InterpolatingDouble;
import SushiFrcLib.Motor.MotorHelper;
import SushiFrcLib.SmartDashboard.TunableNumber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.kHood;

public class Hood extends SubsystemBase {
    private final WPI_TalonFX motor;
    private double pos;

    private final TunableNumber hoodP;
    private final TunableNumber hoodI;
    private final TunableNumber hoodD;
    private final TunableNumber hoodF;
    private final TunableNumber targetPos;

    private static Hood instance;

    public static Hood getInstance() {
        if (instance == null) {
            instance = new Hood();
        }
        return instance;
    }

    private Hood() {
        hoodP = new TunableNumber("Hood P", Constants.kHood.kP, Constants.TUNING_MODE);
        hoodI = new TunableNumber("Hood I", Constants.kHood.kI, Constants.TUNING_MODE);
        hoodD = new TunableNumber("Hood D", Constants.kHood.kD, Constants.TUNING_MODE);
        hoodF = new TunableNumber("Hood F", Constants.kHood.kF, Constants.TUNING_MODE);
        targetPos = new TunableNumber("Target Pos", 0, Constants.TUNING_MODE);

        motor = MotorHelper.createFalconMotor(Ports.HOOD_MOTOR, kHood.CURRENT_LIMIT, kHood.INVERSION, kHood.NEUTRAL_MODE, hoodP.get(), hoodI.get(), hoodD.get(), hoodF.get());
        motor.setSelectedSensorPosition(0);

        pos = 0;
    }

    public void setPos(double newPos) {
        // if (0 <= newPos && newPos <= kHood.maxPos) {
        // pos = newPos;
        // }
        pos = newPos;
    }

    public void setPosBasedOnDistance(double distance) {
        pos = kHood.posMap.getInterpolated(new InterpolatingDouble(distance)).value;
    }

    @Override
    public void periodic() {
        motor.set(ControlMode.Position, pos == 0 ? targetPos.get() : pos);

        SmartDashboard.putNumber("Hood Position", motor.getSelectedSensorPosition());

        if (hoodP.hasChanged()) {
            motor.config_kP(0, hoodP.get());
        }
        
        if (hoodD.hasChanged()) {
            motor.config_kD(0, hoodD.get());
        }
    }
}