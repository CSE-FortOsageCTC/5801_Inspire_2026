package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class IntakeExtensionSubsystem extends SubsystemBase {
    //probably two motors
    // private static TalonFX extensionMaster;
    // private static TalonFX extensionFollower;

    // private DoubleSolenoid intakeExtentionSolenoid;

    private static IntakeExtensionSubsystem intakeExtensionSubsystem;
    private static ClimbSubsystem climbSubsystem;

    private ProfiledPIDController intakePID;

    private double intakeSetpoint = 0;

    private TalonFX extensionMotor;
    
    private Joystick driver;

    private boolean isExtended;

    public static IntakeExtensionSubsystem getInstance(){
        if (intakeExtensionSubsystem == null) {
            intakeExtensionSubsystem = new IntakeExtensionSubsystem();
        }
        return intakeExtensionSubsystem;
    }

    
    private IntakeExtensionSubsystem() {

        // intakeExtentionSolenoid = new DoubleSolenoid(41, PneumaticsModuleType.CTREPCM, 1, 0);
        driver = new Joystick(0);
        isExtended = false;

        climbSubsystem = ClimbSubsystem.getInstance();

        // This is the same motor as what the black wheel kicker was
        extensionMotor = new TalonFX(21);
        extensionMotor.setPosition(0);

        intakePID = new ProfiledPIDController(0.02, 0, 0, new TrapezoidProfile.Constraints(0, 0)); // TODO: Tune :)
        intakePID.setTolerance(0.15);
        intakePID.reset(getIntakeEncoder());

    }

    public void resetExtension(){
        //extensionMaster.set(speed);
        if (!climbSubsystem.getPivotState()){
            // intakeExtentionSolenoid.set(DoubleSolenoid.Value.kForward);
            intakeSetpoint = Constants.maximumIntakeEncoder; // TODO: Get "Zero'd" Position in encoder ticks
            isExtended = false;
        }

    }

    public void setExtension(){
        // intakeExtentionSolenoid.set(DoubleSolenoid.Value.kReverse);
        intakeSetpoint = Constants.minimumIntakeEncoder; // TODO: Get "Extended" Position in encoder ticks
        isExtended = true;
    }

    private void privSetIntake(double speed) {
        extensionMotor.setVoltage(speed * Constants.maximumVoltage);
    }

    public double getIntakeSetpoint() {
        return intakeSetpoint;
    }

    public void setExtensionSetpoint(double setpoint) {

        // Take setpoint set from setExtension/resetExtension methods
        intakeSetpoint = MathUtil.clamp(setpoint, Constants.minimumIntakeEncoder, Constants.maximumIntakeEncoder);

        double calculation = 0;
        intakePID.setGoal(intakeSetpoint);

        if (!intakePID.atGoal()) {
            calculation = intakePID.calculate(getIntakeEncoder());
        } else {
            intakePID.reset(getIntakeEncoder());
        }

        double feed = 0;

        privSetIntake(MathUtil.clamp(feed + calculation, -1, 1));

        SmartDashboard.putNumber("Intake Speed", calculation);
    }

    public double getIntakeEncoder() {
        return extensionMotor.getPosition().getValueAsDouble();
    }

    public boolean getExtensionState() {
        return isExtended;
    }

    @Override
    public void periodic() {

        

        SmartDashboard.putNumber("Intake Extension Encoder", getIntakeEncoder());
    }
}