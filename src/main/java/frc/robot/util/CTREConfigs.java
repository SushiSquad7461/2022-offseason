package frc.robot.util;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import frc.robot.Constants.kSwerve;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;


    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            kSwerve.angleEnableCurrentLimit, 
            kSwerve.angleContinuousCurrentLimit, 
            kSwerve.anglePeakCurrentLimit, 
            kSwerve.anglePeakCurrentDuration);

        swerveAngleFXConfig.slot0.kP = kSwerve.angleKP;
        swerveAngleFXConfig.slot0.kI = kSwerve.angleKI;
        swerveAngleFXConfig.slot0.kD = kSwerve.angleKD;
        swerveAngleFXConfig.slot0.kF = kSwerve.angleKF;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;
        swerveAngleFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;


        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            kSwerve.driveEnableCurrentLimit, 
            kSwerve.driveContinuousCurrentLimit, 
            kSwerve.drivePeakCurrentLimit, 
            kSwerve.drivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = kSwerve.driveKP;
        swerveDriveFXConfig.slot0.kI = kSwerve.driveKI;
        swerveDriveFXConfig.slot0.kD = kSwerve.driveKD;
        swerveDriveFXConfig.slot0.kF = kSwerve.driveKF;        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        swerveDriveFXConfig.openloopRamp = kSwerve.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = kSwerve.closedLoopRamp;

        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = kSwerve.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}