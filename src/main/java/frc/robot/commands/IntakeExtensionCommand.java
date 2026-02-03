package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeExtensionSubsystem;


public class IntakeExtensionCommand extends Command {

    private IntakeExtensionSubsystem intakeExtensionSubsystem;

    public IntakeExtensionCommand() {
        intakeExtensionSubsystem = IntakeExtensionSubsystem.getInstance();
    }

    
    public void open() {
        IntakeExtensionSubsystem.setExtension();
    }

    public void close() {
        intakeExtensionSubsystem.resetExtension();
    }
}
