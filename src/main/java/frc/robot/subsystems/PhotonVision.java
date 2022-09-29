package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;

public class PhotonVision {
    PhotonCamera camera;
    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
    final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
    private static PhotonVision sInstance;

    public static PhotonVision getInstance() {
        if (sInstance == null) {
            sInstance = new PhotonVision();
        }
        return sInstance;
    }

    private PhotonVision() {
        camera = new PhotonCamera("gloworm");
    }

    private List<Double> getAreas() {
        List<PhotonTrackedTarget> result = camera.getLatestResult().getTargets();
        List<Double> ret = new ArrayList<Double>();

        for (var i : result) {
            ret.add(i.getArea());
        }

        return ret;
    }

    private double getBestArea() {
        return camera.getLatestResult().getBestTarget().getArea();
    }

    private double getBestPitch() {
        return camera.getLatestResult().getBestTarget().getPitch();
    }

    private double getBestHeading() {
        return camera.getLatestResult().getBestTarget().getYaw();
    }

    private double getLeftMostHeading() {
        PhotonPipelineResult result = camera.getLatestResult();
        List<PhotonTrackedTarget> targets = result.getTargets();
        if (result.hasTargets()) {
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
