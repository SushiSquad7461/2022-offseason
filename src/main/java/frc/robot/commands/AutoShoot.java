package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class AutoShoot extends CommandBase {
  private final Shooter shooter;
  private final Hood hood;
  private final Indexer indexer;
  private final PhotonVision camera;
  private final Swerve swerve;

  private boolean shoot;
  private double finishDelay;
  private double initTime;

  public AutoShoot() {
    shooter = Shooter.getInstance();
    swerve = Swerve.getInstance();
    indexer = Indexer.getInstance();
    hood = Hood.getInstance();
    camera = PhotonVision.getInstance();

    addRequirements(swerve);
    addRequirements(indexer);
    addRequirements(shooter);
    addRequirements(hood);
    addRequirements(camera);
  }

  @Override
  public void initialize() {
    finishDelay = 0;
    shoot = false;
    initTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    double distance = camera.getDistance();
    shooter.setVelocityBasedOnDistance(distance);
    hood.setPosBasedOnDistance(distance);

    if (shooter.isAtSpeed() && hood.isAtPos() && !shoot) {
      indexer.setShooting();
      shoot = true;
    }
  }

  @Override
  public boolean isFinished() {
    if (Timer.getFPGATimestamp() - initTime > Constants.SHOOT_UP_TO_SPEED_TIMEOUT && !shoot) {
      return true;
    }

    if (shoot) {
      if (finishDelay == 0) {
        finishDelay = Timer.getFPGATimestamp();
        return false;
      } else {
        return Timer.getFPGATimestamp() - finishDelay > 1;
      }
    }
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
    hood.setPos(-1000);
    indexer.setShooting(false);
  }
}
