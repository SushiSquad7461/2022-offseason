package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import SushiFrcLib.Motor.MotorHelper;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import frc.robot.Constants.kIntake;

public class Intake extends SubsystemBase {
    // Motors
    private final WPI_TalonFX pivot;
    private final WPI_TalonFX roller;

    //Subsystem Creation
    private static Intake sInstance = null;

    public static Intake getInstance(String caller) {
        if (sInstance == null) {
            sInstance = new Intake();
        }
        return sInstance;
    }

    private Intake() {
        pivot = MotorHelper.createFalconMotor(Ports.INTAKE_PIVOT, kIntake.PIVOT_CURRENT_LIMIT, TalonFXInvertType.CounterClockwise, NeutralMode.Brake);
        roller = MotorHelper.createFalconMotor(Ports.INTAKE_ROLLER, kIntake.ROLLER_CURRENT_LIMIT, TalonFXInvertType.Clockwise, NeutralMode.Coast);
        pivot.setSelectedSensorPosition(0);
    }

    @Override
    public void periodic() { 
    }

    public void extend() {
        setMotors(kIntake.EXTENDED_SETPOINT, 1);
    }

    public void reverse() {
        setMotors(kIntake.EXTENDED_SETPOINT, -1);
    }

    public void retract() {
        setMotors(0, 0);
    }

    private void setMotors(double pivotGoal, double rollerSpeed) {
        pivot.set(ControlMode.Position, pivotGoal);
        roller.set(ControlMode.PercentOutput, rollerSpeed);
    }
}
