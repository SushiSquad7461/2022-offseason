// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.kSwerveDrive;

import java.util.ArrayList;

import com.swervedrivespecialties.swervelib.SwerveModule;

import SushiFrcLib.Sensors.Gyro.NavX;

public class Swerve extends SubsystemBase {
  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backLeft;
  private final SwerveModule backRight;

  private SwerveModuleState[] moduleStates;

  private final NavX nav = NavX.getInstance(); 

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    // Front left
    new Translation2d(kSwerveDrive.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
        kSwerveDrive.DRIVETRAIN_WHEELBASE_METERS / 2.0),
    // Front right
    new Translation2d(kSwerveDrive.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
        -kSwerveDrive.DRIVETRAIN_WHEELBASE_METERS / 2.0),
    // Back left
    new Translation2d(-kSwerveDrive.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
        kSwerveDrive.DRIVETRAIN_WHEELBASE_METERS / 2.0),
    // Back right
    new Translation2d(-kSwerveDrive.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
        -kSwerveDrive.DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

  private final SwerveDriveOdometry odometry;
  private final Field2d field;

  public Swerve() {
    frontLeft = Constants.kSwerveDrive.FRONT_LEFT.createFourIFalconModule();
    frontRight = Constants.kSwerveDrive.FRONT_RIGHT.createFourIFalconModule();
    backLeft = Constants.kSwerveDrive.BACK_LEFT.createFourIFalconModule();
    backRight = Constants.kSwerveDrive.BACK_RIGHT.createFourIFalconModule();
    odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(nav.getAngle()));
    setChassisSpeedsVelocity(0, 0, 0);
    field = new Field2d();
    SmartDashboard.putData("Field", field);
    nav.zero();
  }

  public Pose2d getPose() {
    return odometry.update(Rotation2d.fromDegrees(nav.getAngle()), moduleStates[0], moduleStates[1], moduleStates[2], moduleStates[3]);
  }

  public void setChassisSpeedsPercentage(double y, double x, double theta) {
    setChassisSpeedsVelocity(
        deadZone(x) * kSwerveDrive.MAX_VELOCITY_METERS_PER_SECOND,
        deadZone(y) * kSwerveDrive.MAX_VELOCITY_METERS_PER_SECOND, 
        deadZone(theta) * kSwerveDrive.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    );
  }

  public double deadZone(double input) {
    return Math.abs(input) > 0.2 ? input : 0;
  }

  public void setChassisSpeedsVelocity(double forwardVelocity, double sidewaysVelocity, double angularVelocity) {
    // this.chassisSpeeds = new ChassisSpeeds(forwardVelocity, sidewaysVelocity, angularVelocity);
    ChassisSpeeds newSpeed= ChassisSpeeds.fromFieldRelativeSpeeds(forwardVelocity, sidewaysVelocity, angularVelocity, Rotation2d.fromDegrees(nav.getAngle()));
    moduleStates = kinematics.toSwerveModuleStates(newSpeed);
  }

  @Override
  public void periodic() {
    SwerveModuleState[] states = moduleStates; // i am lazy

    SwerveDriveKinematics.desaturateWheelSpeeds(states, kSwerveDrive.MAX_VELOCITY_METERS_PER_SECOND);

    SmartDashboard.putNumber("Curr angle  ", nav.getAngle());

    SmartDashboard.putNumber("x position", getPose().getX());
    SmartDashboard.putNumber("y position", getPose().getY());
    SmartDashboard.putNumber("angle", getPose().getRotation().getDegrees());

    updateModules(states);

    field.setRobotPose(getPose());
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public void updateModules(SwerveModuleState[] states){
    frontLeft.set(
        1 * states[0].speedMetersPerSecond / kSwerveDrive.MAX_VELOCITY_METERS_PER_SECOND * kSwerveDrive.MAX_VOLTAGE,
        states[0].angle.getRadians());

    frontRight.set(
        1 * states[1].speedMetersPerSecond / kSwerveDrive.MAX_VELOCITY_METERS_PER_SECOND * kSwerveDrive.MAX_VOLTAGE,
        states[1].angle.getRadians());

    backLeft.set(
        -1 * states[2].speedMetersPerSecond / kSwerveDrive.MAX_VELOCITY_METERS_PER_SECOND * kSwerveDrive.MAX_VOLTAGE,
        states[2].angle.getRadians());

    backRight.set(
        -1 * states[3].speedMetersPerSecond / kSwerveDrive.MAX_VELOCITY_METERS_PER_SECOND * kSwerveDrive.MAX_VOLTAGE,
        states[3].angle.getRadians());
  }
}
