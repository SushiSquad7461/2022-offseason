package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Indexer.IndexerState;

public class Shoot extends CommandBase {
    private final Shooter hooter;
    private final Hood hood;
    private final Indexer indexer;
    private final Swerve swerve;

    private boolean shoot;
    private double finishDelay;
    private double hoodPos;
    private double shooterVelocity;

    public Shoot(double hoodPos, double shooterVelocity) {
        hooter = Shooter.getInstance();
        swerve = Swerve.getInstance();
        indexer = Indexer.getInstance();
        hood = Hood.getInstance();
        this.hoodPos = hoodPos;
        this.shooterVelocity = shooterVelocity;

        addRequirements(swerve);
        addRequirements(indexer);
        addRequirements(hooter);
        addRequirements(hood);
    }

    @Override
    public void initialize() {
        finishDelay = 0;
        shoot = false;
    }

    @Override
    public void execute() {
        hooter.setVelocity(shooterVelocity);
        hood.setPos(hoodPos);

        if (hooter.isAtSpeed() && hood.isAtPos() && !shoot) {
            indexer.setState(IndexerState.SHOOTING);
            shoot = true;
        }
    }

    @Override
    public boolean isFinished() {
        return true;

        // boolean isFinished = shoot;

        // if (isFinished) {
        //     if (finishDelay == 0) {
        //         finishDelay = Timer.getFPGATimestamp();
        //         return false;
        //     } else {
        //         return Timer.getFPGATimestamp() - finishDelay > 1;
        //     }
        // }
        // return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        hooter.stopShooter();
        hood.setPos(-1000);
        indexer.setState(IndexerState.IDLE);
    }
}
