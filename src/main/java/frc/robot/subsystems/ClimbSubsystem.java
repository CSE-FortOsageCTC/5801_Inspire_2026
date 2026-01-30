package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    private static TalonFX masterClimb;
    private static TalonFX followerClimb;
    private static DoubleSolenoid climbPivot;
    private static ClimbSubsystem climbSubsystem;
    private static ProfiledPIDController pidControllerSupported;
    private static ProfiledPIDController pidControllerUnsupported;
    private static double setPoint;
    


    public static ClimbSubsystem getInstance() {
        if (climbSubsystem == null) {
            climbSubsystem = new ClimbSubsystem();
        }
        return (climbSubsystem);
    }

    private ClimbSubsystem() {

        masterClimb = new TalonFX(0);
        followerClimb = new TalonFX(0);

        climbPivot = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 0);

        followerClimb.setControl(new Follower(masterClimb.getDeviceID(), MotorAlignmentValue.Aligned));

        //TODO TUNE THESE
        pidControllerSupported = new ProfiledPIDController(0, 0, 0, null);
        pidControllerUnsupported = new ProfiledPIDController(0, 0, 0, null);
    }

    public double getExtensionEncoder() {
        return masterClimb.getPosition().getValueAsDouble();
    }

    public void setClimbSpeed(double speed) {
        masterClimb.set(speed);
    }

    public void setPivot(){
        climbPivot.set(DoubleSolenoid.Value.kForward);
    }

    public void resetKicker(){
        climbPivot.set(DoubleSolenoid.Value.kReverse);
    }

    public void setPosition(double setPoint){
        this.setPoint = setPoint;

        double speed;
        boolean supported = getExtensionEncoder() - setPoint < 0; //may need to be changed
        //- should be below setpoint, + should be above

        if (supported){
            speed = pidControllerSupported.calculate(getExtensionEncoder(), setPoint);
        }

        else{
            speed = pidControllerUnsupported.calculate(getExtensionEncoder(), setPoint);
        }

        setClimbSpeed(speed);
    }
}