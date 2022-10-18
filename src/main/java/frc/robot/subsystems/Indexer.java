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
import frc.robot.Constants.kPorts;
import frc.robot.Constants.kIndexer;

public class Indexer extends SubsystemBase {
  private final CANSparkMax kicker;
  private final CANSparkMax feeder;
  private final CANSparkMax ejecter;

  private final DigitalInput lowerBeamBreak;
  private final DigitalInput upperBeamBreak;

  private final ColorSensorV3 colorSensor;
  private IndexerState currState;
  private NetworkTable table;

  private boolean isRedAlliance;
  private BallColor ballColor;

  private final Timer timer ;
  private double startTime;
  private double startEjectTime;

  public enum IndexerState {
    WAITING_FOR_COLOR,
    IDLE,
    INTAKING,
    EJECTING,
    MOVING_UP,
    BACKING,
    SHOOTING
  }

  private enum BallColor {
    Unknown,
    Red,
    Blue
  }

  private static Indexer indexer;

  public static Indexer getInstance() {
    if (indexer == null) {
      indexer = new Indexer();
    }
    return indexer;
  }

  /** Creates a new Indexer. */
  private Indexer() {
    table = NetworkTableInstance.getDefault().getTable("FMSInfo");
    isRedAlliance = table.getEntry("IsRedAlliance").getBoolean(true);

    timer = new Timer();
    startEjectTime = 0;
    startTime = 0;

    kicker = MotorHelper.createSparkMax(kPorts.KICKER_MOTOR, MotorType.kBrushless);
    feeder = MotorHelper.createSparkMax(kPorts.FEEDER_MOTOR, MotorType.kBrushless);
    ejecter = MotorHelper.createSparkMax(kPorts.EJECTER_MOTOR, MotorType.kBrushless);

    currState = IndexerState.IDLE;

    colorSensor = new ColorSensorV3(Port.kMXP);
    colorSensor.configureColorSensor(
        ColorSensorResolution.kColorSensorRes13bit,
        ColorSensorMeasurementRate.kColorRate25ms,
        GainFactor.kGain3x);

    lowerBeamBreak = new DigitalInput(kPorts.BOTTOM_BEAM_BREAK);
    upperBeamBreak = new DigitalInput(kPorts.UPPER_BEAM_BREAK);

    timer.start();
  }

  @Override
  public void periodic() {
    isRedAlliance = table.getEntry("IsRedAlliance").getBoolean(true);
    SmartDashboard.putString("Indexer State", currState.toString());

    boolean lowerBeamBreak = lowerBeamBreakActuated();
    boolean upperBeamBreak = upperBeamBreakActuated();

    pollColor();
    boolean correctColor = isCorrectColor();

    switch (currState) {
      case INTAKING:
        if (lowerBeamBreak) {
          setState(IndexerState.WAITING_FOR_COLOR);
        } else {
          break;
        }
        // This case purposefully spills into the waiting for color one
      case WAITING_FOR_COLOR:
        if (ballColor != BallColor.Unknown) {
          if (correctColor) {
            setState(upperBeamBreak
                ? IndexerState.IDLE
                : IndexerState.MOVING_UP);
          } else {
            setState(IndexerState.EJECTING);
          }
        }
        break;
      case EJECTING:
        if (!lowerBeamBreak && timer.get() - startEjectTime > Constants.kIndexer.ejectDelaySeconds) {
          setState(IndexerState.INTAKING);
        }
        break;
      case MOVING_UP:
        if (upperBeamBreak && !lowerBeamBreak) {
          setState(IndexerState.INTAKING);
        }
        break;
      case IDLE:
        break;
      case BACKING:
        break;
      case SHOOTING:
        break;
      default:
        if (!upperBeamBreak && lowerBeamBreak) {
          setState(IndexerState.MOVING_UP);
        } 
        break;
    }
  }

  public boolean lowerBeamBreakActuated() {
    return !lowerBeamBreak.get();
  }

  public boolean upperBeamBreakActuated() {
    return !upperBeamBreak.get();
  }

  public boolean isCorrectColor() {
    //return true;
    return ballColor == BallColor.Red && isRedAlliance || ballColor ==
    BallColor.Blue && !isRedAlliance;
  }

  public void pollColor() {
    Color color = colorSensor.getColor();
    double colorRatio = color.red / color.blue;

    if (colorRatio < 0.75) {
      if (ballColor != BallColor.Blue) {
        System.out.println("blue: " + (timer.get() - startTime));
        System.out.println(colorRatio);
      }
      ballColor = BallColor.Blue;
    } else if (colorRatio > 2.0) {
      if (ballColor != BallColor.Red) {
        System.out.println("red: " + (timer.get() - startTime));
        System.out.println(colorRatio);
      }
      ballColor = BallColor.Red;
    } else {
      if (ballColor != BallColor.Unknown) {
        System.out.println("unknown: " + (timer.get() - startTime));
        System.out.println(colorRatio);
      }
      ballColor = BallColor.Unknown;
    }

    SmartDashboard.putString("Ball Color", ballColor.name());
  }

  public void setIntake() {
    setState(IndexerState.INTAKING);
  }

  public void setIdle() {
    setState(IndexerState.IDLE);
  }

  public void setState(IndexerState newState) {
    if (newState != currState) {
      System.out.println(newState);
    }

    currState = newState;

    switch (currState) {
      case WAITING_FOR_COLOR:
        ejecter.set(0);
        feeder.set(0);
        kicker.set(0);
        break;
      case IDLE:
        ejecter.set(0);
        feeder.set(0);
        kicker.set(0);
        break;
      case INTAKING:
        ejecter.set(0);
        feeder.set(0.5);
        break;
      case EJECTING:
        ejecter.set(-kIndexer.EJECTER_SPEED);
        feeder.set(kIndexer.FEADER_SPEED);
        startEjectTime = timer.get();
        break;
      case MOVING_UP:
        ejecter.set(kIndexer.EJECTER_SPEED);
        feeder.set(kIndexer.FEADER_SPEED);
        break;
      case SHOOTING:
        kicker.set(-kIndexer.KICKER_SPEED * 0.5);
        feeder.set(kIndexer.FEADER_SPEED);
        ejecter.set(kIndexer.EJECTER_SPEED * 0.5);
        break;
      case BACKING:
        // ballCount = 0;
        ejecter.set(kIndexer.EJECTER_SPEED * -1);
        feeder.set(kIndexer.FEADER_SPEED * -1);
      default:
        break;
    }
  }
}
