// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import SushiFrcLib.Motor.MotorHelper;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
    private final WPI_TalonFX rightMotor;

    private static Climb mInstance;

    private boolean goingDown;

    private boolean resetClimb;

    public static Climb getInstance() {
        if (mInstance == null) {
            mInstance = new Climb();
        }
        return mInstance;
    }

    public Climb() {
        rightMotor = MotorHelper.createFalconMotor(Constants.kPorts.RIGHT_CLIMB_MOTOR, Constants.kClimb.CURRENT_LIMIT,
                Constants.kClimb.RIGHT_INVERSION, NeutralMode.Brake);

        rightMotor.setSelectedSensorPosition(0);
        goingDown = false;
        resetClimb = true;
    }

    public void openLoopRaiseClimb() {
        if (getPosition() <= Constants.kClimb.MAX_POS) {
            rightMotor.set(ControlMode.PercentOutput, Constants.kClimb.CLIMB_SPEED * Constants.kClimb.INVERSION);
            goingDown = false;
        }
    }

    public void stop() {
        rightMotor.set(ControlMode.PercentOutput, 0);
    }

    public void openLoopLowerClimb() {
        if (getPosition() >= 0) {
            rightMotor.set(ControlMode.PercentOutput, Constants.kClimb.CLIMB_SPEED * -1 * Constants.kClimb.INVERSION);
            goingDown = true;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("climb current right", rightMotor.getSupplyCurrent());
        SmartDashboard.putNumber("climb pos right", getPosition());

        if (resetClimb) {
            if (rightMotor.getSupplyCurrent() < 0.5) {
                rightMotor.set(ControlMode.PercentOutput, 0.12);
            } else {
                rightMotor.set(ControlMode.PercentOutput, 0);
                rightMotor.setSelectedSensorPosition(0);
                resetClimb = false;
            }
            return;
        }

        if ((getPosition() > Constants.kClimb.MAX_POS && !goingDown) || (getPosition() < 0 && goingDown)) {
            rightMotor.set(ControlMode.PercentOutput, 0);
        }
    }

    public double getPosition() {
        return rightMotor.getSelectedSensorPosition() * -1;
    }

    public void resetClimb() {
        resetClimb = true;
    }
}
