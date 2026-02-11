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

    private static DoubleSolenoid intakeExtentionSolenoid;

    private static IntakeExtensionSubsystem intakeExtensionSubsystem;

    public static IntakeExtensionSubsystem getInstance(){
        if (intakeExtensionSubsystem == null) {
            intakeExtensionSubsystem = new IntakeExtensionSubsystem();
        }
        return intakeExtensionSubsystem;
    }

    
    private IntakeExtensionSubsystem() {

        intakeExtentionSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 0);

        //TODO assign IDs
        // extensionMaster = new TalonFX(0);
        // extensionFollower = new TalonFX(0);
        
        //invert if needed
       // extensionFollower.setControl(new Follower(extensionMaster.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    public static void setExtension(){
        //extensionMaster.set(speed);
        intakeExtentionSolenoid.set(DoubleSolenoid.Value.kForward);
    }
    
    public static void resetExtension(){
        intakeExtentionSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

}