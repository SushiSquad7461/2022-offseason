package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Indexer.IndexerState;

public class AutoShoot extends CommandBase {
  private final Shooter m_shooter;
  private final Hood m_hood;
  private final Indexer m_indexer;
  private final PhotonVision camera;
  private final Swerve m_swerve;
  private boolean shoot;
  private double finishDelay;

  public AutoShoot() {
    m_shooter = Shooter.getInstance();
    m_swerve = Swerve.getInstance();
    m_indexer = Indexer.getInstance();
    m_hood = Hood.getInstance();
    camera = PhotonVision.getInstance();

    addRequirements(m_swerve);
    addRequirements(m_indexer);
    addRequirements(m_shooter);
    addRequirements(m_hood);
  }

  @Override
  public void initialize() {
    finishDelay = 0;
    shoot = false;
  }

  @Override
  public void execute() {
    double distance = camera.getDistance();
    m_shooter.setVelocityBasedOnDistance(distance);
    m_hood.setPosBasedOnDistance(distance);

    if (m_shooter.isAtSpeed() && m_hood.isAtPos() && !shoot) {
      m_indexer.setState(IndexerState.SHOOTING);
      shoot = true;
    }
  }

  @Override
  public boolean isFinished() {
    boolean isFinished = shoot;

    if (isFinished) {
      if (finishDelay == 0) {
        finishDelay = Timer.getFPGATimestamp();
        return false;
      } else {
        return Timer.getFPGATimestamp() - finishDelay > 1;
      }
    }
    return isFinished;
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stopShooter();
    m_hood.setPos(-1000);
    m_indexer.setState(IndexerState.IDLE);
  }
}
