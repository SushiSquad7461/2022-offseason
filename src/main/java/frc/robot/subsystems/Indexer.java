// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3.ColorSensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ColorSensorResolution;
import com.revrobotics.ColorSensorV3.GainFactor;

import SushiFrcLib.Motor.MotorHelper;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.kIndexer;

public class Indexer extends SubsystemBase {
  private final CANSparkMax kicker;
  private final CANSparkMax feeder;
  private final CANSparkMax ejecter;

  private final DigitalInput bottomBeamBreak;
  private final DigitalInput upperBeamBreak;

  private final ColorSensorV3 colorSensor;
  private IndexerState currState;

  private int ballCount;
  private final boolean isRedAlliance;
  private BallColor ballColor;
  // This prevents it from counting the same ball multiple balls
  private boolean canCount = true;


  private final Timer m_timer = new Timer();
  private double m_startTime = 0;


  public enum IndexerState {
    IDLE,
    INTAKING,
    EJECTING,
    STORING,
    SHOOTING,
    MOVING_UP
  }

  private enum BallColor {
    Unknown,
    Red,
    Blue
  }

  private static Indexer indexer = null;
  public static Indexer getInstance() {
    if( indexer == null) {
      indexer = new Indexer();
    }
    return indexer;
  }

  /** Creates a new Indexer. */
  private Indexer() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("FMSInfo");
    isRedAlliance = table.getEntry("IsRedAlliance").getBoolean(true);

    kicker = MotorHelper.createSparkMax(Constants.Ports.KICKER_MOTOR, MotorType.kBrushless);
    feeder = MotorHelper.createSparkMax(Ports.FEEDER_MOTOR, MotorType.kBrushless);
    ejecter = MotorHelper.createSparkMax(Ports.EJECTER_MOTOR, MotorType.kBrushless);

    currState = IndexerState.IDLE;

    colorSensor = new ColorSensorV3(Port.kOnboard);
    colorSensor.configureColorSensor(
      ColorSensorResolution.kColorSensorRes13bit,
      ColorSensorMeasurementRate.kColorRate25ms,
      GainFactor.kGain3x
    );

    bottomBeamBreak = new DigitalInput(Ports.BOTTOM_BEAM_BREAK);
    upperBeamBreak = new DigitalInput(Ports.UPPER_BEAM_BREAK);

    ballCount = 0;

    m_timer.start();
  }

  public boolean canIntake() {
    return ballCount < 2;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Ejecter", ejecter.getAppliedOutput());
    SmartDashboard.putNumber("Kicker", kicker.getAppliedOutput());
    SmartDashboard.putNumber("Feeder", feeder.getAppliedOutput());
    SmartDashboard.putString("Indexer State", currState.toString());
    SmartDashboard.putBoolean("Beam Break", bottomBeamBreak.get());

    pollColor();

    if (lowerBeamBreakActuated()) {
      if (ballColor == BallColor.Unknown && currState == IndexerState.INTAKING) { 
        setState(IndexerState.IDLE);
      } else if (isCorrectColor() && canCount) {
        ballCount += 1;
        canCount = false;
        setState(canIntake() ? IndexerState.MOVING_UP : IndexerState.IDLE);
      } else if (!isCorrectColor()) {
        setState(IndexerState.EJECTING);
      }
    } else if ((currState == IndexerState.EJECTING && !lowerBeamBreakActuated()) || (currState == IndexerState.MOVING_UP && !upperBeamBreakActuated())) {
      setState(IndexerState.INTAKING);
      canCount = true;
    }
  }

  public boolean lowerBeamBreakActuated() {
    return !bottomBeamBreak.get();
  }

  public boolean upperBeamBreakActuated() {
    return !upperBeamBreak.get();
  }

  public boolean isCorrectColor() {
    return ballColor == BallColor.Red && isRedAlliance || ballColor == BallColor.Blue && !isRedAlliance;
  }

  public void pollColor() {
    Color color = colorSensor.getColor();
    
    if(color.blue > Constants.kIndexer.colorSensorThreasholdBlue) {
      if (ballColor != BallColor.Blue) {
        System.out.println("blue: " + (m_timer.get() - m_startTime));
      }
      ballColor = BallColor.Blue;
    } else if(color.red > Constants.kIndexer.colorSensorThreasholdRed) {
      if(ballColor != BallColor.Red) {
        System.out.println("red: " + (m_timer.get() - m_startTime));
      } 
      ballColor = BallColor.Red;
    } else {
      if(ballColor != BallColor.Unknown) {
        System.out.println("unknown: " + (m_timer.get() - m_startTime));
      } 
      ballColor = BallColor.Unknown;
    }

    SmartDashboard.putString("Ball Color", ballColor.name());
  }

  public void setIntake() {
    setState(IndexerState.INTAKING);
  }

  public void setState(IndexerState newState) {
    if (newState != currState) {
      System.out.println(newState);
    }
    currState = newState;

    switch (currState) {
      case IDLE:
        kicker.set(0);
        ejecter.set(0);
        feeder.set(0);
        break;
      case INTAKING:
        kicker.set(0);
        ejecter.set(0);
        feeder.set(0.5); // diffrent from other feeder speeds, is this needed?
        break;
      case EJECTING:
        kicker.set(0);
        ejecter.set(-kIndexer.EJECTER_SPEED);
        feeder.set(kIndexer.FEADER_SPEED);
        break;
      case MOVING_UP:
        kicker.set(kIndexer.KICKER_SPEED);
        ejecter.set(kIndexer.EJECTER_SPEED);
        feeder.set(kIndexer.FEADER_SPEED);
        break;
      default:
        break;
    }
  }
}
