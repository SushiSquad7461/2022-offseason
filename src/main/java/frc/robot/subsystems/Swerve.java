package frc.robot.subsystems;

import SushiFrcLib.Kinematics.SwerveDriveKinematics;
import SushiFrcLib.SwerveModule.SwerveModuleSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kSwerve;

public class Swerve extends SubsystemBase {
    private SwerveModuleSparkMax frontRight;
    private SwerveModuleSparkMax frontLeft;
    private SwerveModuleSparkMax backLeft;
    private SwerveModuleSparkMax backRight;
    private SwerveDriveKinematics kinematics;
    
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
        kinematics = new SwerveDriveKinematics(kSwerve.WHEEL_BASE, kSwerve.MAX_SPEED);
    }

    @Override
    public void periodic() { 
        synchronized (Swerve.this) {
            kinematics.calculate(, )
        }
    }


    public void updateDriveValues(double x, double y, double turn) {
        this.x = x;
        this.y = y;
        this.turn = turn;
    }
}
