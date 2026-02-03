package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSubsystem;

public class L1Climb extends Command{
    
    private ClimbSubsystem climbSubsystem;
    private boolean extensionUp = false;
    

    
    public L1Climb(){
        climbSubsystem = climbSubsystem.getInstance();

        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize(){
        climbSubsystem.setPivot();
    }

    @Override
    public void execute(){
        
        if (!extensionUp && climbSubsystem.getExtensionEncoder() >= Constants.primaryL1ClimbSetpoint){ //past setpoint, ready to go down
            extensionUp = true;
        }

        if (!extensionUp){ //extend
            climbSubsystem.setPosition(Constants.primaryL1ClimbSetpoint);
        }

        else { //retract
            climbSubsystem.setPosition(Constants.secondaryL1ClimbSetpoint);
        }
    }

    @Override
    public void end(boolean isFinished){
        climbSubsystem.setClimbSpeed(0);
    }

}