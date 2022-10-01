package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Indexer.IndexerState;

public class AutoShoot extends CommandBase{
    private final Shooter m_shooter;
    private final Hood m_hood;
    private final Indexer m_indexer;
    private final double shooterSpeed;
    private final double hoodPos;
    private boolean shoot;

    public AutoShoot(double speed, double pos) {
        shooterSpeed = speed;
        hoodPos = pos;
        m_shooter = Shooter.getInstance();
        m_indexer = Indexer.getInstance();
        m_hood = Hood.getInstance();
        shoot = false;
    }

    @Override
    public void execute() {
        m_shooter.setVelocity(shooterSpeed);
        m_hood.setPos(hoodPos);
        if( m_shooter.isAtSpeed() && m_hood.isAtPos() && !shoot) {
            m_indexer.setShooting();
            shoot = true;
        }
    }

    @Override
    public boolean isFinished() {
        return shoot && !m_indexer.getShooting();
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stopShooter();
        m_indexer.setState(IndexerState.IDLE);
    }
}
