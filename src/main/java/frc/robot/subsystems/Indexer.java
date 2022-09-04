// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.net.CacheRequest;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import SushiFrcLib.Motor.MotorHelper;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Ports;

public class Indexer extends SubsystemBase {
  CANSparkMax kicker;
  CANSparkMax feeder;
  CANSparkMax ejecter;

  DigitalInput bottomBeamBreak = new DigitalInput(0);
  DigitalInput topBeamBreak = new DigitalInput(1);

  ColorSensorV3 colorSensor;
  I2C i2C = new i2C

  IndexerState currState;

  int ballCount;

  enum IndexerState {
    IDLE,
    INTAKING,
    EJECTING,
    STORING,
    SHOOTING,
    MOVING_UP
  }

  /** Creates a new Indexer. */
  public Indexer() {
    kicker = MotorHelper.createSparkMax(Constants.Ports.KICKER_MOTOR, MotorType.kBrushless);
    feeder = MotorHelper.createSparkMax(Ports.FEEDER_MOTOR, MotorType.kBrushless);
    ejecter = MotorHelper.createSparkMax(Ports.EJECTER_MOTOR, MotorType.kBrushless);
    currState = IndexerState.IDLE;
    colorSensor = new ColorSensorV3();
    ballCount = 0;
  }

  public boolean canIntake() {
    return ballCount < 2;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Ejecter", ejecter.getAppliedOutput());
    SmartDashboard.putNumber("Kicker", kicker.getAppliedOutput());
    SmartDashboard.putNumber("Feeder", feeder.getAppliedOutput());

    if (currState == IndexerState.INTAKING && lowerBeamBreakActuated()) { 
      if (isBallRight()) {
        ballCount += 1;
        setState(canIntake() ? IndexerState.MOVING_UP : IndexerState.IDLE);
      } else {
        setState(IndexerState.EJECTING);
      }
    } else if ((currState == IndexerState.EJECTING && !lowerBeamBreakActuated()) || (currState == IndexerState.MOVING_UP && upperBeamBreakActuated())) {
      setState(IndexerState.INTAKING);
    }
  }

  public boolean lowerBeamBreakActuated() {
    return !bottomBeamBreak.get();
  }

  public boolean upperBeamBreakActuated() {
    return false;
  }

  public boolean isBallRight() {
    return true;
  }

  public void setIntake() {
    setState(IndexerState.INTAKING);
  }

  public void setState(IndexerState newState) {
    currState = newState;

    switch (currState) {
      case IDLE:
        kicker.set(0);
        feeder.set(0);
        ejecter.set(0);
        break;
      case INTAKING:
        kicker.set(0);
        ejecter.set(0);
        feeder.set(0.8);
        break;
      case EJECTING:
        kicker.set(0);
        ejecter.set(-1);
        feeder.set(0.8);
        break;
      case MOVING_UP:
        kicker.set(1);
        ejecter.set(1);
        feeder.set(0.8);
        break;
      default:
        break;
    }
  }
}
