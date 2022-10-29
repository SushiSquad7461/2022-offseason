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
            kSwerve.ANGLE_ENABLE_CURRENT_LIMIT, 
            kSwerve.ANGLE_CONTINUSE_CURRENT_LIMIT, 
            kSwerve.ANGLE_PEAK_CURRENT_LIMIT, 
            kSwerve.ANGLE_PEAK_CURRENT_DURATION);

        swerveAngleFXConfig.slot0.kP = kSwerve.ANGLE_P;
        swerveAngleFXConfig.slot0.kI = kSwerve.ANGLE_I;
        swerveAngleFXConfig.slot0.kD = kSwerve.ANGLE_D;
        swerveAngleFXConfig.slot0.kF = kSwerve.ANGLE_F;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;
        swerveAngleFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;


        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            kSwerve.DRIVE_ENABLE_CURRENT_LIMIT, 
            kSwerve.DRIVE_CONTINUSE_CURRENT_LIMIT, 
            kSwerve.DRIVE_PEAK_CURRENT_LIMIT, 
            kSwerve.DRIVE_PEAK_CURRENT_LIMIT);

        swerveDriveFXConfig.slot0.kP = kSwerve.DRIVE_P;
        swerveDriveFXConfig.slot0.kI = kSwerve.DRIVE_I;
        swerveDriveFXConfig.slot0.kD = kSwerve.DRIVE_D;
        swerveDriveFXConfig.slot0.kF = kSwerve.DRIVE_F;        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        swerveDriveFXConfig.openloopRamp = kSwerve.OPEN_LOOP_RAMP;
        swerveDriveFXConfig.closedloopRamp = kSwerve.CLOSED_LOOP_RAMP;

        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = kSwerve.CANCODER_INVERSION;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}