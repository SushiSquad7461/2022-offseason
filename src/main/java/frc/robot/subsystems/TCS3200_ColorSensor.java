package frc.robot.subsystems;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.Queue;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Robot;
import frc.robot.subsystems.TCS3200_ColorReader.ColorSelect;
import frc.robot.subsystems.TCS3200_ColorReader.FreqScaling;

public class TCS3200_ColorSensor implements Sendable {
  private static final double MIN_COUNT_TIME = 0.003;

  private final TCS3200_ColorReader m_reader;

  private final Queue<ColorSelect> m_colorsToRead;

  private double m_red;
  private double m_green;
  private double m_blue;
  private double m_clear;

  public TCS3200_ColorSensor(int out, int s0, int s1, int s2, int s3, FreqScaling startFreq, ColorSelect... colorsToRead) {
    m_colorsToRead = new LinkedList<>();
    m_colorsToRead.addAll(Arrays.asList(colorsToRead));

    m_reader = new TCS3200_ColorReader(out, s0, s1, s2, s3, ColorSelect.RED, startFreq);

    Robot.getInstance().addPeriodic(() -> this.update(), MIN_COUNT_TIME);

    SendableRegistry.addLW(this, getClass().getSimpleName());
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

  public Color getColor() {
    double mag = m_clear;

    return new Color(m_red / mag, m_green / mag, m_blue / mag);
  }

  public void update() {
    m_reader.update();

    switch (m_reader.getOutputColor()) {
      case RED:
        m_red = m_reader.get();
        break;
      case GREEN:
        m_green = m_reader.get();
        break;
      case BLUE:
        m_blue = m_reader.get();
        break;
      case CLEAR:
        m_clear = m_reader.get();
        break;
    }

    ColorSelect newColor = m_colorsToRead.remove();
    m_reader.setOutputColor(newColor);
    m_colorsToRead.add(newColor);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("TCS3200 Color Sensor");

    builder.addDoubleProperty("red", this::getRed, null);
    builder.addDoubleProperty("green", this::getGreen, null);
    builder.addDoubleProperty("blue", this::getBlue, null);
    builder.addDoubleProperty("clear", this::getClear, null);
    builder.addStringProperty("color", () -> {
      Color color = this.getColor();
      return String.format("%.3f, %.3f, %.3f", color.red, color.green, color.blue);
    }, null);
  }
}
