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

    private ProfiledPIDController downIntakePID;
    private ProfiledPIDController upIntakePID;

    private double intakeSetpoint = 0;

    private TalonFX extensionMotor;

    private boolean isExtended;

    private boolean isJiggling = false;

    public static IntakeExtensionSubsystem getInstance(){
        if (intakeExtensionSubsystem == null) {
            intakeExtensionSubsystem = new IntakeExtensionSubsystem();
        }
        return intakeExtensionSubsystem;
    }

    
    private IntakeExtensionSubsystem() {

        // intakeExtentionSolenoid = new DoubleSolenoid(41, PneumaticsModuleType.CTREPCM, 1, 0);
        isExtended = false;

        climbSubsystem = ClimbSubsystem.getInstance();

        // This is the same motor as what the black wheel kicker was
        extensionMotor = new TalonFX(21);
        extensionMotor.setPosition(0);

        downIntakePID = new ProfiledPIDController(0.03, 0, 0, new TrapezoidProfile.Constraints(0, 0)); // TODO: Tune :)
        downIntakePID.setTolerance(0.05);
        downIntakePID.reset(getIntakeEncoder());

        upIntakePID = new ProfiledPIDController(0.04, 0, 0, new TrapezoidProfile.Constraints(0, 0)); // TODO: Tune :)
        upIntakePID.setTolerance(0.05);
        upIntakePID.reset(getIntakeEncoder());

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

    public void setIsJiggling(boolean jiggling) {
        isJiggling = jiggling;
        if (!jiggling) {
            intakeSetpoint = Constants.minimumIntakeEncoder;
        }
        
    }

    public boolean getIsJiggling() {
        return isJiggling;
    }

    public void setExtensionSetpoint(double setpoint) {

        // Take setpoint set from setExtension/resetExtension methods
        intakeSetpoint = MathUtil.clamp(setpoint, Constants.minimumIntakeEncoder, Constants.maximumIntakeEncoder);

        double downCalculation = 0;
        double upCalculation = 0;
        downIntakePID.setGoal(intakeSetpoint);
        upIntakePID.setGoal(intakeSetpoint);

        if (!downIntakePID.atGoal()) {
            downCalculation = downIntakePID.calculate(getIntakeEncoder());
            upCalculation = upIntakePID.calculate(getIntakeEncoder());
        } else {
            downIntakePID.reset(getIntakeEncoder());
            upIntakePID.reset(getIntakeEncoder());
        }

        double feed = 0;
        double calculation = intakeSetpoint > getIntakeEncoder() ? upCalculation : downCalculation;

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

        // Always call ts to update position/always calculate
        // setExtensionToSetpoint();

        SmartDashboard.putNumber("Intake Extension Encoder", getIntakeEncoder());
    }
}