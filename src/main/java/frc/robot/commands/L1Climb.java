package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSubsystem;

public class L1Climb extends Command{
    
    private ClimbSubsystem climbSubsystem;
    private double setPoint;
    

    
    public L1Climb(double setPoint){
        climbSubsystem = climbSubsystem.getInstance();

        this.setPoint = setPoint;

        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize(){
        climbSubsystem.setPivot();
    }

    @Override
    public void execute(){
        
        if (climbSubsystem.getExtensionEncoder() - setPoint < 0){ //below setpoint
            climbSubsystem.setPosition(Constants.maxClimbExtension);
        }

        else { //above setpoint
            climbSubsystem.setPosition(Constants.minClimbExtension);
        }
    }

    @Override
    public void end(boolean isFinished){
        climbSubsystem.setClimbSpeed(0);
    }

}