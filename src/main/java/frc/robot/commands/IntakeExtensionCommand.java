package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeExtensionSubsystem;


public class IntakeExtensionCommand extends Command {

    private IntakeExtensionSubsystem intakeExtensionSubsystem;

    private boolean hasRun = false;

    public IntakeExtensionCommand() {

        intakeExtensionSubsystem = IntakeExtensionSubsystem.getInstance();

        addRequirements(intakeExtensionSubsystem);
    }

    @Override
    public void execute() {
        if (intakeExtensionSubsystem.getExtensionState()) {
            System.out.println("Reset Extension");
            intakeExtensionSubsystem.resetExtension();
        } else {
            System.out.println("Set Extenstion");
            intakeExtensionSubsystem.setExtension();
        }
        hasRun = true;        
    }
    
    @Override
    public boolean isFinished() {
        return hasRun;
    }
}
