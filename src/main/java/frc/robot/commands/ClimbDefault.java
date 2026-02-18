package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbDefault extends Command{
    private Joystick operator;

    public ClimbSubsystem climbSubsystem;

    public ClimbDefault(Joystick operator){
        this.operator = operator;
        this.climbSubsystem = ClimbSubsystem.getInstance();

        addRequirements(climbSubsystem);
    }

    @Override
    public void execute(){

        if (operator.getRawButton(XboxController.Button.kLeftBumper.value)){ //up
            climbSubsystem.setClimbSpeed(0.1); //low for safety
        }

        else if (operator.getRawButton(XboxController.Button.kRightBumper.value)){ //down
            climbSubsystem.setClimbSpeed(-0.1);
        }
    }
}