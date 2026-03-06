package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;

public class IntakeExtensionSubsystem extends SubsystemBase {
    //probably two motors
    // private static TalonFX extensionMaster;
    // private static TalonFX extensionFollower;

    private DoubleSolenoid intakeExtentionSolenoid;

    private static IntakeExtensionSubsystem intakeExtensionSubsystem;
    private static ClimbSubsystem climbSubsystem;

    private boolean isExtended;

    public static IntakeExtensionSubsystem getInstance(){
        if (intakeExtensionSubsystem == null) {
            intakeExtensionSubsystem = new IntakeExtensionSubsystem();
        }
        return intakeExtensionSubsystem;
    }

    
    private IntakeExtensionSubsystem() {

        intakeExtentionSolenoid = new DoubleSolenoid(41, PneumaticsModuleType.CTREPCM, 1, 0);

        isExtended = false;

        climbSubsystem = ClimbSubsystem.getInstance();

        //TODO assign IDs
        // extensionMaster = new TalonFX(0);
        // extensionFollower = new TalonFX(0);
        
        //invert if needed
       // extensionFollower.setControl(new Follower(extensionMaster.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    public void setExtension(){
        //extensionMaster.set(speed);
        if (!climbSubsystem.getPivotState()){
            intakeExtentionSolenoid.set(DoubleSolenoid.Value.kForward);
            isExtended = true;
        }

    }
    
    public void resetExtension(){
        intakeExtentionSolenoid.set(DoubleSolenoid.Value.kReverse);
        isExtended = false;
    }

    public boolean getExtensionState() {
        return isExtended;
    }
}