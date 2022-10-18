package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import SushiFrcLib.Math.Conversion;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kVision;

public class PhotonVision extends SubsystemBase {
    private final PhotonCamera camera;

    private PhotonPipelineResult result;
    private PhotonTrackedTarget bestTarget;
    private boolean hasTargets;
    private boolean lastHeadingPositive;

    private static PhotonVision instance;

    public static PhotonVision getInstance() {
        if (instance == null) {
            instance = new PhotonVision();
        }
        return instance;
    }

    private PhotonVision() {
        camera = new PhotonCamera("gloworm");
        lastHeadingPositive = true;
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

        double angle = kVision.LIME_LIGHT_MOUNT_ANGLE + getBestPitch();
        return (1 / Math.tan(Conversion.degreesToRadians(angle))) * kVision.LIME_LIGHT_TO_HUB_HEIGHT;
    }
}
