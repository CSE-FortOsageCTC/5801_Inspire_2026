package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command{
    
    private IntakeSubsystem intakeSubsystem;
    public boolean intaking;
    public double topSpeed = 0.3;
    public double bottomSpeed = 0.4;

    public IntakeCommand(boolean intaking) {
        this.intaking = intaking;

        intakeSubsystem = IntakeSubsystem.getInstance();

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setIntaking(true);
    }

    @Override
    public void execute(){
        if (intaking) {
            intakeSubsystem.setIntakeSpeed(topSpeed, -bottomSpeed);
        }
        else {
            intakeSubsystem.setIntakeSpeed(-topSpeed, bottomSpeed);
        }
        
    }

    @Override
    public boolean isFinished() {
        return !intakeSubsystem.isIntaking();
    }

    @Override
    public void end(boolean isFinished) {
        intakeSubsystem.setIntaking(false);
        intakeSubsystem.setIntakeSpeed(0, 0);
    }
}
