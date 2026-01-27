package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command{
    
    private IntakeSubsystem intakeSubsystem;
    public boolean intaking;
    public double speed=1;

    public IntakeCommand(boolean intaking) {
        this.intaking = intaking;

        intakeSubsystem = IntakeSubsystem.getInstance();

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute(){
        if (intaking) {
            intakeSubsystem.setIntakeSpeed(speed);
        }
        else {
            intakeSubsystem.setIntakeSpeed(-speed);
        }
        
    }

    @Override
    public void end(boolean isFinished) {
        intakeSubsystem.setIntakeSpeed(0);
    }
}
