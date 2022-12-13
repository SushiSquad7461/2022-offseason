package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import SushiFrcLib.CheesyLibUtil.InterpolatingDouble;
import SushiFrcLib.Motor.MotorHelper;
import SushiFrcLib.SmartDashboard.TunableNumber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.kPorts;
import frc.robot.Constants.kHood;

public class Hood extends SubsystemBase {
    private final WPI_TalonFX motor;

    private final TunableNumber hoodP;
    private final TunableNumber hoodD;
    private final TunableNumber targetPos;
    private static Hood instance;
    private boolean reset;

    public static Hood getInstance() {
        if (instance == null) {
            instance = new Hood();
        }
        return instance;
    }

    private Hood() {
        hoodP = new TunableNumber("Hood P", kHood.kP, Constants.TUNING_MODE);
        hoodD = new TunableNumber("Hood D", kHood.kD, Constants.TUNING_MODE);
        targetPos = new TunableNumber("Target Pos", 0, Constants.TUNING_MODE);

        motor = MotorHelper.createFalconMotor(kPorts.HOOD_MOTOR, kHood.CURRENT_LIMIT, 
            kHood.INVERSION, kHood.NEUTRAL_MODE, hoodP.get(), kHood.kI, hoodD.get(), kHood.kF);

        reset = true;
    }

    public void setPos(double newPos) {
        targetPos.setDefault(newPos);
    }

    public void setPosBasedOnDistance(double distance) {
        targetPos.setDefault(
            kHood.POS_MAP.getInterpolated(new InterpolatingDouble(distance)).value
            + Constants.kHood.OFFSET
        );
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hood Position", motor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Hood Error", motor.getClosedLoopError());
        SmartDashboard.putNumber("Hood Current", motor.getSupplyCurrent());

        if (reset) {
            setMotor(ControlMode.PercentOutput, kHood.TENSION_SPEED);

            if (motor.getSupplyCurrent() >= kHood.TENSION_CURRENT) {
                setMotor(ControlMode.PercentOutput, 0);
                motor.setSelectedSensorPosition(0);
                reset = false;
            }
        } else {
            // Stops motor from moving up if tunable numbers has high number or low number
            if (targetPos.get() < kHood.MAX_POS && targetPos.get() > kHood.MIN_POS) {
                setMotor(ControlMode.Position, targetPos.get());
            }

            if (hoodP.hasChanged()) {
                motor.config_kP(0, hoodP.get());
            }

            if (hoodD.hasChanged()) {
                motor.config_kD(0, hoodD.get());
            }   
        }
    }

    private void setMotor(ControlMode controlMode, double speed) {
        motor.set(controlMode, speed);
    }

    public boolean isAtPos() {
        return Math.abs(targetPos.get() - motor.getSelectedSensorPosition()) < kHood.HOOD_ERROR;
    }

    public void reset(){
        reset = true;
    }
}
