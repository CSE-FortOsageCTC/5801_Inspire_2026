package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SpindexerCommand extends Command{
    
    private ShooterSubsystem shooterSubsystem;
    public boolean intaking;
    public double speed=0.1;

    public SpindexerCommand(boolean intaking) {
        this.intaking = intaking;

        shooterSubsystem = ShooterSubsystem.getInstance();

        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute(){
        if (intaking) {
            shooterSubsystem.setSpindexer(-speed);
        }
        else {
            shooterSubsystem.setSpindexer(speed);
        }
        
    }

    @Override
    public void end(boolean isFinished) {
        shooterSubsystem.setSpindexer(0);
    }
}
