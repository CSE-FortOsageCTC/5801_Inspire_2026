package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.Follower;

public class ClimbSubsystem extends SubsystemBase {
    private static TalonFX MasterClimb;
    private static TalonFX FollowerClimb;
    private static ClimbSubsystem climbSubsystem;
    private static ProfiledPIDController pidController;


    public static ClimbSubsystem getInstance() {
        if (climbSubsystem == null) {
            climbSubsystem = new ClimbSubsystem();
        }
        return (climbSubsystem);
    }

    private ClimbSubsystem() {

        MasterClimb = new TalonFX(0);
        FollowerClimb = new TalonFX(0);

        FollowerClimb.setControl(new Follower(MasterClimb.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    public double getExtensionEncoder() {
        return MasterClimb.getPosition().getValueAsDouble();
    }

    public void setClimbSpeed(double speed) {
        MasterClimb.set(speed);

    }
}