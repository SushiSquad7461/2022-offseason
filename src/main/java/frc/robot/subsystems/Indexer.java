// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.net.CacheRequest;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import SushiFrcLib.Motor.MotorHelper;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.kIndexer;

public class Indexer extends SubsystemBase {
  CANSparkMax kicker;
  CANSparkMax feeder;
  CANSparkMax ejecter;
  private static Indexer indexer = null;

  DigitalInput bottomBeamBreak = new DigitalInput(0);
  DigitalInput topBeamBreak = new DigitalInput(1);

  ColorSensorV3 colorSensor;
  IndexerState currState;

  int ballCount;
  boolean isRedAlliance;
  boolean isRed = false;
  boolean isBlue = false;

  enum IndexerState {
    IDLE,
    INTAKING,
    EJECTING,
    STORING,
    SHOOTING,
    MOVING_UP
  }

  public static Indexer getInstance() {
    if( indexer == null) {
      indexer = new Indexer();
    }
    return indexer;
  }

  /** Creates a new Indexer. */
  private Indexer() {
    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    NetworkTable table = instance.getTable("FMSInfo");
    isRedAlliance = table.getEntry("IsRedAlliance").getBoolean(true);
    kicker = MotorHelper.createSparkMax(Constants.Ports.KICKER_MOTOR, MotorType.kBrushless);
    feeder = MotorHelper.createSparkMax(Ports.FEEDER_MOTOR, MotorType.kBrushless);
    ejecter = MotorHelper.createSparkMax(Ports.EJECTER_MOTOR, MotorType.kBrushless);
    currState = IndexerState.IDLE;
    colorSensor = new ColorSensorV3(Port.kOnboard);
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
    SmartDashboard.putString("Indexer State", currState.toString());
    SmartDashboard.putBoolean("Beam Break", bottomBeamBreak.get());
    pollColor();
    if (currState == IndexerState.INTAKING && lowerBeamBreakActuated()) {
      if (isCorrectColor()) {
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

  public boolean isCorrectColor() {
    return isRed && isRedAlliance || isBlue && !isRedAlliance;
  }

  public void pollColor() {
    int red = colorSensor.getRed();
    int blue = colorSensor.getBlue();
    SmartDashboard.putBoolean("Red", isRed);
    SmartDashboard.putBoolean("Blue", isBlue);
    //int green = colorSensor.getGreen();
    if((double)red/blue > kIndexer.colorSensorThreasholdRed) {
      isRed = true;
      isBlue = false;
    } else if((double)blue/red > kIndexer.colorSensorThreasholdBlue) {
      isBlue = true;
      isRed = false;
    }

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
