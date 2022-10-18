package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import SushiFrcLib.Math.Conversion;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PhotonVision extends SubsystemBase {
    private final PhotonCamera camera;
    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
    public static final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);

    private PhotonPipelineResult result;
    private PhotonTrackedTarget bestTarget;
    private boolean hasTargets;
    private boolean lastHeadingPositive = true;

    private static PhotonVision sInstance;

    public static PhotonVision getInstance() {
        if (sInstance == null) {
            sInstance = new PhotonVision();
        }
        return sInstance;
    }

    @Override
    public void periodic() {
        result = camera.getLatestResult();
        bestTarget = result.getBestTarget();
        hasTargets = result.hasTargets();
        if (hasTargets) {
            lastHeadingPositive = bestTarget.getYaw() > 0;
        }
        SmartDashboard.putNumber("Heading", getBestHeading());
    }

    private PhotonVision() {
        camera = new PhotonCamera("gloworm");
    }

    private double getBestArea() {
        return hasTargets ? bestTarget.getArea() : 0;
    }

    private double getBestPitch() {
        return hasTargets ? bestTarget.getPitch() : 0;
    }

    private double getBestHeading() {
        return hasTargets ? bestTarget.getYaw() : (lastHeadingPositive ? 30 : -30);
    }

    private double getAvreageHeading() {
        if (hasTargets) {
            List<PhotonTrackedTarget> targets = result.getTargets();
            double sum = 0;

            for (var target : targets) {
                sum += target.getYaw();
            }

            System.out.println(sum / targets.size());

            return -(sum / targets.size());
        }
        return (lastHeadingPositive ? 30 : -30);
    }

    private double getLeftMostHeading() {
        if (hasTargets) {
            List<PhotonTrackedTarget> targets = result.getTargets();
            double LeftestYaw = targets.get(0).getYaw();

            for (var target : targets) {
                if (target.getYaw() < LeftestYaw) {
                    LeftestYaw = target.getYaw();
                }
            }

            return LeftestYaw;
        }
        return 0;
    }

    public double getHeading() {
        return getAvreageHeading();
    }

    public double getDistance() {
        if (!camera.getLatestResult().hasTargets()) {
            return 0;
        }

        double angle = Constants.VisionConstants.kLimeLightMountAngle + getBestPitch();
        return (1 / Math.tan(Conversion.degreesToRadians(angle))) * Constants.VisionConstants.kLimeLightToHubHeight;
    }
}
