package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SpindexerCommand extends Command{
    
    private ShooterSubsystem shooterSubsystem;
    public boolean feeding;
    public double spinSpeed = 0.4;
    public double kickerSpeed = 0.1;

    public SpindexerCommand(boolean feeding) {
        this.feeding = feeding;

        shooterSubsystem = ShooterSubsystem.getInstance();

        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute(){
        if (feeding) {
            shooterSubsystem.setSpindexer(spinSpeed, kickerSpeed);
        }
        else {
            shooterSubsystem.setSpindexer(-spinSpeed, kickerSpeed);
        }
        
    }

    @Override
    public void end(boolean isFinished) {
        shooterSubsystem.setSpindexer(0, 0);
    }
}
