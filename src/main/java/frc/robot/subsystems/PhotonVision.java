package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import SushiFrcLib.Math.Conversion;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kVision;

public class PhotonVision extends SubsystemBase {
    private final PhotonCamera camera;
    private final LinearFilter distanceFilter;

    private PhotonPipelineResult result;
    private boolean lastHeadingPositive;
    private double distance;

    private static PhotonVision instance;

    public static PhotonVision getInstance() {
        if (instance == null) {
            instance = new PhotonVision();
        }
        return instance;
    }

    private PhotonVision() {
        camera = new PhotonCamera(kVision.CAMERA_NAME);
        distanceFilter = LinearFilter.singlePoleIIR(kVision.TIME_CONSTANT, kVision.PERIOD);

        distance = 0;
        lastHeadingPositive = true;
    }

    @Override
    public void periodic() {
        result = camera.getLatestResult();
        distance = distanceFilter.calculate(calculateDistance());

        SmartDashboard.putNumber("Distance", distance);

        if (result.hasTargets()) {
            lastHeadingPositive = result.getBestTarget().getYaw() > 0;
        }
    }

    private double getBestPitch() {
        return result.hasTargets() ? result.getBestTarget().getPitch() : 0;
    }

    private double getAverageHeading() {
        if (result.hasTargets()) {
            List<PhotonTrackedTarget> targets = result.getTargets();
            double sum = 0;

            for (var target : targets) {
                sum += target.getYaw();
            }

            return -(sum / targets.size());
        }
        return (lastHeadingPositive ? 30 : -30);
    }

    public double getHeading() {
        return getAverageHeading();
    }

    public double calculateDistance() {
        if (!camera.getLatestResult().hasTargets()) {
            return 0;
        }

        double angle = kVision.LIME_LIGHT_MOUNT_ANGLE + getBestPitch();
        return (1 / Math.tan(Conversion.degreesToRadians(angle))) * kVision.LIME_LIGHT_TO_HUB_HEIGHT;
    }
    
    public double getDistance() {
        return distance;
    }
}
