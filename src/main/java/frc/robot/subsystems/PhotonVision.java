package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
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
            ret.add(i.getYaw());
        }

        return ret;
    }

    private double getBestArea() {
        return camera.getLatestResult().getBestTarget().getArea();
    }

    public double getDistance() {
        return getBestArea();
    }
}
