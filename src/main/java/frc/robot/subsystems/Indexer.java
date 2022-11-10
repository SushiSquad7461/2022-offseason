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
  private boolean colorEnabled;

  private final DigitalInput lowerBeamBreak;
  private final DigitalInput upperBeamBreak;

  private final ColorSensorV3 colorSensor;
  private IndexerState currState;
  private NetworkTable table;

  private int ballCount = 0;
  private boolean isRedAlliance;
  private BallColor ballColor;
  // This prevents it from counting the same ball multiple balls
  private boolean isShooting = false;
  private boolean overrideIdle = false;

  private final Timer m_timer = new Timer();
  private double m_startTime = 0;
  private double m_startEjectTime = 0;
  private double m_startMovingUpTime = 0;

  public enum IndexerState {
    WAITING_FOR_BALL,
    INTAKING,
    EJECTING,
    MOVING_UP,
    BACKING,
    SHOOTING,
    AUTO_INTAKE
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

    kicker = MotorHelper.createSparkMax(kPorts.KICKER_MOTOR, MotorType.kBrushless);
    feeder = MotorHelper.createSparkMax(kPorts.FEEDER_MOTOR, MotorType.kBrushless);
    ejecter = MotorHelper.createSparkMax(kPorts.EJECTER_MOTOR, MotorType.kBrushless);

    currState = IndexerState.WAITING_FOR_BALL;

    colorSensor = new ColorSensorV3(Port.kMXP);
    colorSensor.configureColorSensor(
        ColorSensorResolution.kColorSensorRes13bit,
        ColorSensorMeasurementRate.kColorRate25ms,
        GainFactor.kGain3x);

    lowerBeamBreak = new DigitalInput(kPorts.BOTTOM_BEAM_BREAK);
    upperBeamBreak = new DigitalInput(kPorts.UPPER_BEAM_BREAK);

    ballCount = 0;
    colorEnabled = true;

    m_timer.start();
  }

  public boolean canIntake() {
    return ballCount < 2;
  }

  @Override
  public void periodic() {
    isRedAlliance = table.getEntry("IsRedAlliance").getBoolean(true);
    SmartDashboard.putString("Indexer State", currState.toString());

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
      case EJECTING:
        if (!lowerBeamBreak && m_timer.get() - m_startEjectTime > Constants.kIndexer.ejectDelaySeconds) {
          ballCount--;
          setState(IndexerState.INTAKING);
        }
        break;
      case MOVING_UP:
        if (upperBeamBreak && !lowerBeamBreak
          && m_timer.get() - m_startMovingUpTime > Constants.kIndexer.movingUpDelaySeconds) {
          setState(IndexerState.INTAKING);
        }
        break;
      case INTAKING:
        if (!lowerBeamBreak) {
          if (overrideIdle) {
            setState(IndexerState.WAITING_FOR_BALL);
          }
          break;
        } 

      // This case purposefully spills into the waiting for color one
      default:
      case WAITING_FOR_BALL:
        if (ballColor == BallColor.Unknown) {
          if (canIntake() && !overrideIdle) {
            setState(IndexerState.INTAKING);
          } else {
            setState(IndexerState.WAITING_FOR_BALL);
          }
        } else {
          if (correctColor) {
            setState(upperBeamBreak
                ? IndexerState.WAITING_FOR_BALL
                : IndexerState.MOVING_UP);
          } else {
            setState(IndexerState.EJECTING);
          }
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
      kicker.set(-kIndexer.KICKER_SPEED);
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
    //return true;
    return ballColor == BallColor.Red && isRedAlliance || ballColor ==
    BallColor.Blue && !isRedAlliance;
  }

  public void enableColor (boolean enable){
    colorEnabled = enable;
  }

  public void pollColor() {
    Color color = colorSensor.getColor();
    double colorRatio = color.red / color.blue;

    if (colorRatio < Constants.kIndexer.colorSensorRatioThresholdBlue) {
      if (ballColor != BallColor.Blue) {
        System.out.println("blue: " + (m_timer.get() - m_startTime));
        System.out.println(colorRatio);
      }
      ballColor = BallColor.Blue;
    } else if (colorRatio > Constants.kIndexer.colorSensorRatioThresholdRed) {
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

    SmartDashboard.putNumber("ball ratio", colorRatio);
    SmartDashboard.putString("Ball Color", ballColor.name());
  }

  private void setState(IndexerState newState) {
    if (newState != currState) {
      System.out.println(newState);
    }

    currState = newState;

    switch (currState) {
      case WAITING_FOR_BALL:
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
        feeder.set(kIndexer.FEEDER_SPEED);
        m_startEjectTime = m_timer.get();
        break;
      case MOVING_UP:
        ejecter.set(kIndexer.EJECTER_SPEED);
        feeder.set(kIndexer.FEEDER_SPEED);
        break;
      default:
        break;
    }
  }
}
