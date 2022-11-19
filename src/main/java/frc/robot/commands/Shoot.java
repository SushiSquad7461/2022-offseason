package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Indexer.IndexerState;

public class Shoot extends CommandBase {
    private final Shooter shooter;
    private final Hood hood;
    private final Indexer indexer;
    private final Swerve swerve;

    private boolean shoot;
    private double shootTime;
    private double initTime;
    private double hoodPos;
    private double shooterVelocity;

    public Shoot(double hoodPos, double shooterVelocity) {
        shooter = Shooter.getInstance();
        indexer = Indexer.getInstance();
        hood = Hood.getInstance();
        swerve = Swerve.getInstance();

        this.hoodPos = hoodPos;
        this.shooterVelocity = shooterVelocity;

        addRequirements(indexer);
        addRequirements(shooter);
        addRequirements(hood);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        shootTime = 0;
        shoot = false;
        initTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        shooter.setVelocity(shooterVelocity);
        hood.setPos(hoodPos);

        if (shooter.isAtSpeed() && hood.isAtPos() && !shoot) {
            indexer.setState(IndexerState.SHOOTING);
            shoot = true;
        }
    }

    @Override
    public boolean isFinished() {
        if (Timer.getFPGATimestamp() - initTime > Constants.SHOOT_UP_TO_SPEED_TIMEOUT && !shoot) {
            return true;
        }

        if (shoot) {
            if (shootTime == 0) {
                shootTime = Timer.getFPGATimestamp();
                return false;
            } else {
                return Timer.getFPGATimestamp() - shootTime > Constants.SHOOT_TIMEOUT;
            }
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooter();
        hood.setPos(-1000);
        indexer.setState(IndexerState.IDLE);
    }
}
