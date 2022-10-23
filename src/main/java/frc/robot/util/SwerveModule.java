package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.kPorts;
import frc.robot.Constants.kSwerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import java.util.PriorityQueue;

import SushiFrcLib.SmartDashboard.TunableNumber;

public class SwerveModule {
    public int moduleNumber;
    private double angleOffset;
    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANCoder angleEncoder;
    private double lastAngle;
    private PriorityQueue<Double> cancoderAngles;

    private final TunableNumber mDriveP;
    private final TunableNumber mDriveD;
    private final TunableNumber mDriveF;

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        cancoderAngles = new PriorityQueue<Double>(5);
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        angleEncoder = new WPI_CANCoder(moduleConstants.cancoderID, kPorts.CANIVORE_NAME);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new WPI_TalonFX(moduleConstants.angleMotorID, kPorts.CANIVORE_NAME);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new WPI_TalonFX(moduleConstants.driveMotorID, kPorts.CANIVORE_NAME);
        configDriveMotor();

        lastAngle = getState().angle.getDegrees();

        mDriveP = new TunableNumber("Mod " + moduleNumber + " drive P", 0.0, Constants.TUNING_MODE);
        mDriveD = new TunableNumber("Mod " + moduleNumber + " drive D", 0.0, Constants.TUNING_MODE);
        mDriveF = new TunableNumber("Mod " + moduleNumber + " drive F", 0.0, Constants.TUNING_MODE);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (mDriveP.hasChanged()) {
            mDriveMotor.config_kP(0, mDriveP.get());
        }
        if (mDriveD.hasChanged()) {
            mDriveMotor.config_kD(0, mDriveD.get());
        }
        if (mDriveF.hasChanged()) {
            mDriveMotor.config_kF(0, mDriveF.get());
        }

        desiredState = CTREModuleState.optimize(desiredState, getState().angle); // Custom optimize command, since
                                                                                 // default WPILib optimize assumes
                                                                                 // continuous controller which CTRE is
                                                                                 // not

        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / kSwerve.MAX_SPEED;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        } else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, kSwerve.WHEEL_CIRCUMFRANCE,
                    kSwerve.DRIVE_GEAR_RATIO);
            mDriveMotor.set(ControlMode.Velocity, velocity);
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (kSwerve.MAX_SPEED * 0.01)) ? lastAngle
                : desiredState.angle.getDegrees(); // Prevent rotating module if speed is less then 1%. Prevents
                                                   // Jittering.
        mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle, kSwerve.ANGLE_GEAR_RATIO));
        lastAngle = angle;
    }

    public void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToFalcon(getAngle(), kSwerve.ANGLE_GEAR_RATIO);
        // for( Double d : )
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder() {
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor() {
        mAngleMotor.configFactoryDefault();
        mAngleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        mAngleMotor.setInverted(kSwerve.ANGLE_INVERSION);
        mAngleMotor.setNeutralMode(kSwerve.ANGLE_NEUTRAL_MODE);
        resetToAbsolute();
    }

    private void configDriveMotor() {
        mDriveMotor.configFactoryDefault();
        mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setInverted(kSwerve.DRIVE_INVERSION);
        mDriveMotor.setNeutralMode(kSwerve.DRIVE_NEUTRAL_MODE);
        mDriveMotor.setSelectedSensorPosition(0);
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public double getAngle() {
        return -getCanCoder().getDegrees() + angleOffset;
    }

    public SwerveModuleState getState() {
        double velocity = Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), kSwerve.WHEEL_CIRCUMFRANCE,
                kSwerve.DRIVE_GEAR_RATIO);
        Rotation2d angle = Rotation2d.fromDegrees(
                Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), kSwerve.ANGLE_GEAR_RATIO));
        return new SwerveModuleState(velocity, angle);
    }

}