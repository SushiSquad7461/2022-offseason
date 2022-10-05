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

  private final DigitalInput lowerBeamBreak;
  private final DigitalInput upperBeamBreak;

  private final ColorSensorV3 colorSensor;
  private IndexerState currState;

  private int ballCount = 0;
  private final boolean isRedAlliance;
  private BallColor ballColor;
  // This prevents it from counting the same ball multiple balls
  private boolean isShooting = false;
  private boolean overrideIdle = false;

  private final Timer m_timer = new Timer();
  private double m_startTime = 0;
  private double m_startEjectTime = 0;
  private double m_startMovingUpTime = 0;

  public enum IndexerState {
    WAITING_FOR_COLOR,
    IDLE,
    INTAKING,
    EJECTING,
    MOVING_UP
  }

  private enum BallColor {
    Unknown,
    Red,
    Blue
  }

  private static Indexer indexer = null;

  public static Indexer getInstance() {
    if (indexer == null) {
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

    colorSensor = new ColorSensorV3(Port.kMXP);
    colorSensor.configureColorSensor(
        ColorSensorResolution.kColorSensorRes13bit,
        ColorSensorMeasurementRate.kColorRate25ms,
        GainFactor.kGain3x);

    lowerBeamBreak = new DigitalInput(Ports.BOTTOM_BEAM_BREAK);
    upperBeamBreak = new DigitalInput(Ports.UPPER_BEAM_BREAK);

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
    SmartDashboard.putNumber("Ball count", ballCount);
    // SmartDashboard.putBoolean("Beam Break", bottomBeamBreak.get());

    boolean lowerBeamBreak = lowerBeamBreakActuated();
    boolean upperBeamBreak = upperBeamBreakActuated();

    pollColor();
    boolean correctColor = isCorrectColor();

    ballCount = 0;
    if (lowerBeamBreak) {
      ballCount++;
    } if (upperBeamBreak) {
      ballCount++;
    }

    if (isShooting && !upperBeamBreak) {
      setShooting(false);
    }

    switch (currState) {
      case INTAKING:
        if (!canIntake()) {
          setState(IndexerState.IDLE);
          break;
        } else if (!lowerBeamBreak) {
          break;
        } 

        // This case purposefully spills into the waiting for color one
      case WAITING_FOR_COLOR:
        if (ballColor == BallColor.Unknown) {
          setState(IndexerState.WAITING_FOR_COLOR);
        } else {
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
        if (!lowerBeamBreak && m_timer.get() - m_startEjectTime > Constants.kIndexer.ejectDelaySeconds) {
          setState(IndexerState.INTAKING);
        }
        break;
      case MOVING_UP:
        if (upperBeamBreak && !lowerBeamBreak
          && m_timer.get() - m_startMovingUpTime > Constants.kIndexer.movingUpDelaySeconds) {
          setState(IndexerState.INTAKING);
        }
        break;
      case IDLE:
      default:
        if (canIntake()) {
          if (!upperBeamBreak && lowerBeamBreak) {
            setState(IndexerState.MOVING_UP);
          } else {
            if (!overrideIdle) {
              setState(IndexerState.INTAKING);
            } else {
              setState(IndexerState.IDLE);
            }
          }
        } else {
          setState(IndexerState.IDLE);
        }
        break;
    }

    // if (currState == IndexerState.SHOOTING && !upperBeamBreak) {
    // setState(IndexerState.IDLE);
    // ballCount = 0;
    // } else if (!canIntake()) {
    // setState(IndexerState.IDLE);
    // } else if (lowerBeamBreak) {
    // pollColor();
    // if (ballColor == BallColor.Unknown && currState == IndexerState.INTAKING) {
    // setState(IndexerState.IDLE);
    // } else if (correctColor && canCount) {
    // ballCount += 1;
    // canCount = false;
    // setState(correctColor ? IndexerState.MOVING_UP : IndexerState.IDLE);
    // } else if (!isCorrectColor()) {
    // setState(IndexerState.EJECTING);
    // }
    // } else if ((currState == IndexerState.EJECTING && !lowerBeamBreak) ||
    // (currState == IndexerState.MOVING_UP && !upperBeamBreak)) {
    // setState(IndexerState.INTAKING);
    // canCount = true;
    // }
  }

  public void setShooting() {
    setShooting(true);
  }

  public void setShooting(boolean shooting) {
    isShooting = shooting;
    if (shooting) {
      kicker.
      set(-kIndexer.KICKER_SPEED);
    } else {
      kicker.set(0);
    }
    SmartDashboard.putBoolean("isShooting", isShooting);
  }

  public boolean getShooting() {
    return isShooting;
  }

  public void setOverrideIdle(boolean value) {
    overrideIdle = value;
  }

  public boolean getOverrideIdle() {
    return overrideIdle;
  }

  public boolean lowerBeamBreakActuated() {
    return !lowerBeamBreak.get();
  }

  public boolean upperBeamBreakActuated() {
    return !upperBeamBreak.get();
  }

  public boolean isCorrectColor() {
    return ballColor == BallColor.Red && isRedAlliance || ballColor == BallColor.Blue && !isRedAlliance;
  }

  public void pollColor() {
    Color color = colorSensor.getColor();
    double colorRatio = color.red / color.blue;

    if (colorRatio < 0.75) {
      if (ballColor != BallColor.Blue) {
        System.out.println("blue: " + (m_timer.get() - m_startTime));
        System.out.println(colorRatio);
      }
      ballColor = BallColor.Blue;
    } else if (colorRatio > 2.0) {
      if (ballColor != BallColor.Red) {
        System.out.println("red: " + (m_timer.get() - m_startTime));
        System.out.println(colorRatio);
      }
      ballColor = BallColor.Red;
    } else {
      if (ballColor != BallColor.Unknown) {
        System.out.println("unknown: " + (m_timer.get() - m_startTime));
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
      case IDLE:
        ejecter.set(0);
        feeder.set(0);
        break;
      case INTAKING:
        ejecter.set(0);
        feeder.set(0.5); // diffrent from other feeder speeds, is this needed?
        break;
      case EJECTING:
        ejecter.set(-kIndexer.EJECTER_SPEED);
        feeder.set(kIndexer.FEADER_SPEED);
        m_startEjectTime = m_timer.get();
        break;
      case MOVING_UP:
        ejecter.set(kIndexer.EJECTER_SPEED);
        feeder.set(kIndexer.FEADER_SPEED);
        m_startMovingUpTime = m_timer.get();
        break;
      default:
        break;
    }
  }
}
