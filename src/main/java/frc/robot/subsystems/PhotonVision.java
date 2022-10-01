package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;

public class PhotonVision {
    PhotonCamera camera;
    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
    public static final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);

    private PhotonPipelineResult result;
    private PhotonTrackedTarget bestTarget;
    private boolean hasTargets;

    private static PhotonVision sInstance;
    public static PhotonVision getInstance() {
        if (sInstance == null) {
            sInstance = new PhotonVision();
        }
        return sInstance;
    }
    
    // @Override
    public void periodic() {
        result = camera.getLatestResult();
        bestTarget = result.getBestTarget();
        hasTargets = result.hasTargets();
    }

    private PhotonVision() {
        camera = new PhotonCamera("gloworm");
    }

    private double getBestArea() {
        return hasTargets ? bestTarget.getArea() : 0;
    }

    private double getBestPitch() {
        return hasTargets ? bestTarget.getPitch(): 0;
    }

    private double getBestHeading() {
        return hasTargets ? bestTarget.getYaw(): 0;
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
        //return getLeftMostHeading(); //TODO: switch to this if not working
        return getBestHeading();
    }

    public double getDistance() {
        //return getBestArea(); //TODO: switch to this if not working
        return getBestPitch();
    }
}
