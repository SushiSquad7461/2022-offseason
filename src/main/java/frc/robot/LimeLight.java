package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLight {
    private static LimeLight m_instance;
    
    private final NetworkTableEntry m_tv;
    private final NetworkTableEntry m_tx;
    private final NetworkTableEntry m_ty;

    private LimeLight() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

        m_tv = table.getEntry("tv");
        m_tx = table.getEntry("tx");
        m_ty = table.getEntry("ty");
    }

    public static LimeLight getInstance() {
        return m_instance == null ? new LimeLight() : m_instance;
    }

    public double getHeading() {
        return m_tx.getDouble(0.0);
    }

    // Based off https://docs.limelightvision.io/en/latest/cs_estimating_distance.html#using-a-fixed-angle-camera
    public double getDistance() {
        double ty = m_ty.getDouble(0.0);

        double angleToGoalDegrees = Constants.Vision.kLimeLightMountAngleDegrees + ty;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

        return (Constants.Vision.kHubTargetHeightMeters - Constants.Vision.kLimeLightLensHeightMeters)
            / Math.tan(angleToGoalRadians);
    }

    public boolean canSeeTarget() {
        return m_tv.getDouble(0.0) == 1.0;
    }
}
