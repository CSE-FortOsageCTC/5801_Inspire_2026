package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class IntakeExtensionSubsystem {
    //probably two motors
    private static TalonFX extensionMaster;
    private static TalonFX extensionFollower;

    private static IntakeExtensionSubsystem intakeExtensionSubsystem;

    public static IntakeExtensionSubsystem getInstance(){
        if (intakeExtensionSubsystem == null) {
            intakeExtensionSubsystem = new IntakeExtensionSubsystem();
        }
        return intakeExtensionSubsystem;
    }

    private IntakeExtensionSubsystem() {

        
        //TODO assign IDs
        extensionMaster = new TalonFX(0);
        extensionFollower = new TalonFX(0);
        
        //invert if needed
        extensionFollower.setControl(new Follower(extensionMaster.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    public void setExtensionSpeed(double speed){
        extensionMaster.set(speed);
    }
}