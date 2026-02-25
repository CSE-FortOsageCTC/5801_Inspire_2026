package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    
    private static TalonFX intakeMaster;
    private static TalonFX intakeFollower;

    private static IntakeSubsystem intakeSubsystem;

    public static IntakeSubsystem getInstance(){
        if (intakeSubsystem == null) {
            intakeSubsystem = new IntakeSubsystem();
        }
        return intakeSubsystem;
    }

    private IntakeSubsystem() {

        //TODO assign IDs
        intakeMaster = new TalonFX(23);
        intakeFollower = new TalonFX(24);
        
        //invert if needed
        intakeFollower.setControl(new Follower(intakeMaster.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public void setIntakeSpeed(double speed){
        intakeMaster.set(speed);
    }
    
    public void addInstruments(Orchestra orchestra){
        orchestra.addInstrument(intakeMaster);
        orchestra.addInstrument(intakeFollower);
    }
}
