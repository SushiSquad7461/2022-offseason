package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TCS3200_ColorSensor extends SubsystemBase {
  // Output managment
  private final Counter m_counter;

  // Configurable registers
  private final DigitalOutput m_s0;
  private final DigitalOutput m_s1;
  private final DigitalOutput m_s2;
  private final DigitalOutput m_s3;

  private FreqScaling m_freqScaling;
  private ColorSelect m_currColor;

  private double m_lastTimeStamp;

  public enum FreqScaling {
    POWER_DOWN(0),
    PERCENT2(1),
    PERCENT20(2),
    PERCENT100(3);

    public final int val;

    FreqScaling(int i) {
      val = i;
    }
  }

  public enum ColorSelect {
    RED(0),
    GREEN(1),
    BLUE(2),
    CLEAR(3);

    public final int val;

    ColorSelect(int i) {
      val = i;
    }
  }

  private double m_red;
  private double m_green;
  private double m_blue;
  private double m_clear;

  public TCS3200_ColorSensor(int out, int s0, int s1, int s2, int s3, ColorSelect startColor) {
    m_counter = new Counter(out);

    // To save DIO ports, let user not set some
    m_s0 = s0 == -1 ? null : new DigitalOutput(s0);
    m_s1 = s1 == -1 ? null : new DigitalOutput(s1);
    m_s2 = s2 == -1 ? null : new DigitalOutput(s2);
    m_s3 = s3 == -1 ? null : new DigitalOutput(s3);

    setFreqScaling(FreqScaling.PERCENT100);
    setOutputColor(startColor);

    m_lastTimeStamp = Timer.getFPGATimestamp();
  }

  public void setFreqScaling(FreqScaling scale) {
    m_freqScaling = scale;

    if (m_s0 == null || m_s1 == null) {
      return;
    }

    // Check the data sheet pg 2 for the truth table
    // https://www.mouser.com/catalog/specsheets/tcs3200-e11.pdf
    switch (scale) {
      case POWER_DOWN:
        m_s0.set(false);
        m_s1.set(false);
      case PERCENT2:
        m_s0.set(false);
        m_s1.set(true);
      case PERCENT20:
        m_s0.set(true);
        m_s1.set(false);
      case PERCENT100:
        m_s0.set(true);
        m_s1.set(true);
    }

    m_counter.reset();
  }

  public FreqScaling getFreqScaling() {
    return m_freqScaling;
  }

  public void setOutputColor(ColorSelect color) {
    m_currColor = color;

    if (m_s2 == null || m_s3 == null) {
      return;
    }

    // Check the data sheet pg 2 for the truth table
    // https://www.mouser.com/catalog/specsheets/tcs3200-e11.pdf
    switch (color) {
      case RED:
        m_s2.set(false);
        m_s3.set(false);
        break;
      case BLUE:
        m_s2.set(false);
        m_s3.set(true);
        break;
      case CLEAR:
        m_s2.set(true);
        m_s3.set(false);
        break;
      case GREEN:
        m_s2.set(true);
        m_s3.set(true);
        break;
    }

    m_counter.reset();
  }

  public ColorSelect getOutputColor() {
    return m_currColor;
  }

  public double getGreen() {
    return m_green;
  }

  public double getRed() {
    return m_red;
  }

  public double getBlue() {
    return m_blue;
  }

  public double getClear() {
    return m_clear;
  }

  @Override
  public void periodic() {

    double time = Timer.getFPGATimestamp();

    m_red = 0;
    m_blue = 0;
    m_green = 0;
    m_clear = 0;

    // Scale the number counted to amount of time that passed since last check
    double freq = (m_counter.get() / (time - m_lastTimeStamp));
    switch (m_currColor) {
      case RED:
        m_red = freq;
        break;
      case GREEN:
        m_green = freq;
        break;
      case BLUE:
        m_blue = freq;
        break;
      case CLEAR:
        m_clear = freq;
        break;
    }

    m_counter.reset();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addStringProperty("Frequency Scaling", () -> this.getFreqScaling().toString(), null);
    builder.addStringProperty("Current Color", () -> this.getOutputColor().toString(), null);
    builder.addDoubleProperty("Red", this::getRed, null);
    builder.addDoubleProperty("Green", this::getGreen, null);
    builder.addDoubleProperty("Blue", this::getBlue, null);
    builder.addDoubleProperty("Clear", this::getClear, null);
  }
}