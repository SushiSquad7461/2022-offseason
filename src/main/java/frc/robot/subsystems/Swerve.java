package frc.robot.subsystems;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;

import SushiFrcLib.Kinematics.SwerveDriveKinematics;
import SushiFrcLib.Math.Rotation2;
import SushiFrcLib.Math.Vector2;
import SushiFrcLib.SwerveModule.SwerveModuleSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kSwerve;
import edu.wpi.first.wpilibj.SPI;

public class Swerve extends SubsystemBase {
    private SwerveModuleSparkMax frontRight;
    private SwerveModuleSparkMax frontLeft;
    private SwerveModuleSparkMax backLeft;
    private SwerveModuleSparkMax backRight;
    private SwerveDriveKinematics kinematics;
    private final AHRS nav; 
    
    //Subsystem Creation
    private static Swerve sInstance = null;

    private double x;
    private double y;
    private double turn;

    public static Swerve getInstance(String caller) {
        if (sInstance == null) {
            sInstance = new Swerve(caller);
        }
        return sInstance;
    }

    private Swerve(String caller) {
        frontRight = new SwerveModuleSparkMax(kSwerve.frontRight);
        frontLeft = new SwerveModuleSparkMax(kSwerve.frontLeft);
        backRight = new SwerveModuleSparkMax(kSwerve.backRight);
        backLeft = new SwerveModuleSparkMax(kSwerve.backLeft);

        frontRight.start();
        frontLeft.start();
        backRight.start();
        backLeft.start();
        kinematics = new SwerveDriveKinematics(kSwerve.WHEEL_BASE/2.0, kSwerve.MAX_SPEED);
        nav = new AHRS(SPI.Port.kMXP);
        nav.enableBoardlevelYawReset(false);
    }

    @Override
    public void periodic() { 
        synchronized (Swerve.this) {
            frontRight.periodic();
            frontLeft.periodic();
            backRight.periodic();
            backLeft.periodic();
        }
    }


    public void updateDriveValues(double x, double y, double turn) {
        this.x = x;
        this.y = y;
        this.turn = turn;
        ArrayList<Vector2> moduleStates = kinematics.calculate(Rotation2.fromRadians(nav.getRotation2d().getRadians()), x, y, turn);
        frontRight.updateModule(moduleStates.get(0));
        frontLeft.updateModule(moduleStates.get(1));
        backRight.updateModule(moduleStates.get(2));
        backLeft.updateModule(moduleStates.get(3));
    }
}
