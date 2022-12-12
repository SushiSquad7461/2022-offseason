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
import frc.robot.Constants.kClimb;
import frc.robot.Constants.kPorts;

public class Climb extends SubsystemBase {
    private final WPI_TalonFX motor;

    private boolean goingDown;
    private boolean resetClimb;

    private static Climb instance;

    public static Climb getInstance() {
        if (instance == null) {
            instance = new Climb();
        }
        return instance;
    }

    public Climb() {
        motor = MotorHelper.createFalconMotor(kPorts.CLIMB_MOTOR, kClimb.CURRENT_LIMIT,
                kClimb.INVERSION, NeutralMode.Brake);

        goingDown = false;
        resetClimb = true;
    }

    public void openLoopRaiseClimb() {
        if (getPosition() <= kClimb.MAX_POS) {
            setMotor(ControlMode.PercentOutput, kClimb.CLIMB_SPEED);
            goingDown = false;
        }
    }

    public void stop() {
        setMotor(ControlMode.PercentOutput, 0);
    }

    public void openLoopRetractClimb() {
        if (getPosition() >= 0) {
            setMotor(ControlMode.PercentOutput, kClimb.CLIMB_LOWER_SPEED);
            goingDown = true;
        }
    }

    private void setMotor(ControlMode controlMode, double speed) {
        motor.set(controlMode, speed);
    }

    @Override
    public void periodic() {
        double climbPosition = getPosition();
        SmartDashboard.putNumber("climb current right", motor.getSupplyCurrent());
        SmartDashboard.putNumber("climb pos right", climbPosition);

        if (resetClimb) {
            if (motor.getSupplyCurrent() < kClimb.TENSION_CURRENT) {
                setMotor(ControlMode.PercentOutput, kClimb.STARTUP_LOWER_SPEED);
            } else {
                setMotor(ControlMode.PercentOutput, 0);
                motor.setSelectedSensorPosition(0);
                resetClimb = false;
            }
        } else if ((climbPosition > kClimb.MAX_POS && !goingDown) || (climbPosition < 0 && goingDown)) {
            setMotor(ControlMode.PercentOutput, 0);
        }
    }

    public double getPosition() {
        return motor.getSelectedSensorPosition();
    }

    public void resetClimb() {
        resetClimb = true;
    }
}
