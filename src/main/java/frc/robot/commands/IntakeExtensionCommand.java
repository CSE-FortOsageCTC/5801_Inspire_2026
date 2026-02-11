package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeExtensionSubsystem;


public class IntakeExtensionCommand extends Command {

    private IntakeExtensionSubsystem intakeExtensionSubsystem;

    private boolean isExtending;

    private boolean hasRun = false;

    public IntakeExtensionCommand(boolean isExtending) {
        this.isExtending = isExtending;

        intakeExtensionSubsystem = IntakeExtensionSubsystem.getInstance();

        addRequirements(intakeExtensionSubsystem);
    }

    @Override
    public void execute() {
        if (isExtending) {
            IntakeExtensionSubsystem.setExtension();
        }
        else {
            IntakeExtensionSubsystem.resetExtension();
        }
        hasRun = true;
    }
    
    @Override
    public boolean isFinished() {
        return hasRun;
    }
}
