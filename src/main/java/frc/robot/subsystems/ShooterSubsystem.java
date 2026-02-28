// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private static ShooterSubsystem shooterSubsystem;
  
  private SparkMax swivel;
  private TalonFX flywheelMaster;
  private TalonFX flywheelFollower;
  private TalonFX spindexerMaster;
  // private TalonFX spindexerFollower;
  private TalonFX turretHood;
  private TalonFX kicker;
  //TODO: Add kicker motor

  public double swivelSetpoint = 0;
  public double turretHoodSetpoint = 0;

  private ProfiledPIDController swivelPID;
  private ProfiledPIDController hoodPID;
  
  private static boolean isShooting = false;
 
  public static ShooterSubsystem getInstance(){
        if (shooterSubsystem == null){
            shooterSubsystem = new ShooterSubsystem();
        } 
        return (shooterSubsystem);
    }
  private ShooterSubsystem() {
    //TODO assign IDs 
    // flywheelMaster = new TalonFX(0);
    // flywheelFollower = new TalonFX(0);
    swivel = new SparkMax(55, MotorType.kBrushless);
    spindexerMaster = new TalonFX(22);
    // spindexerFollower = new TalonFX(22);
    turretHood = new TalonFX(0);
    kicker = new TalonFX(13);
    //Change configs as need be
    
    /* Only need this logic if we are using an absolute encoder (which we just got rid of) */
    // try {
    //   getSwivelAbsoluteEncoder();
    //   getSwivelEncoder();
    //   Thread.sleep(200);
    // } catch (InterruptedException e) {
    //   e.printStackTrace();
    // }
    
    
    swivel.getEncoder().setPosition(0); // Make sure turret is hitting hard stop :)

    // flywheelFollower.setControl(new Follower(flywheelMaster.getDeviceID(), MotorAlignmentValue.Aligned));
    // spindexerFollower.setControl(new Follower(spindexerMaster.getDeviceID(), MotorAlignmentValue.Aligned));

    swivelPID = new ProfiledPIDController(0.08, 0.001, 0.001, new TrapezoidProfile.Constraints(500, 500));
    swivelPID.setTolerance(0.75);

    hoodPID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
    hoodPID.setTolerance(0);

    swivelSetpoint = getSwivelEncoder();
    turretHoodSetpoint = getHoodEncoder();

    swivelPID.reset(swivelSetpoint);
    hoodPID.reset(turretHoodSetpoint);

  }

  public void setFlywheels(double speed){
    flywheelMaster.set(speed);
  }

  public void setKicker(double speed) {
    kicker.set(speed);
  }

  private void privSetSwivel(double speed) {
    swivel.set(speed);
  }
  public double getSwivelSetpoint() {
    return swivelSetpoint;
  }
  public void setSwivelSetpoint(double setpoint) {
    // if (setpoint != swivelSetpoint) {
    //   swivelPID.reset(getSwivelEncoder());
    // }

    swivelSetpoint = setpoint;

    double calculation = 0;
    swivelPID.setGoal(setpoint);
    // double pidValue = swivelPID.calculate(getSwivelEncoder());

    if (!swivelPID.atGoal()) {
      calculation = swivelPID.calculate(getSwivelEncoder());
    } else {
      swivelPID.reset(getSwivelEncoder());
    }

    // if (!swivelPID.atSetpoint()) {
    //   privSetSwivel(MathUtil.clamp(calculation, -1, 1));
    // }

    privSetSwivel(MathUtil.clamp(calculation, -1, 1));

    SmartDashboard.putNumber("Swivel Speed", calculation);
    
    
  }

  public double getSwivelEncoder() {
    return swivel.getEncoder().getPosition();
  }

  public double getSwivelAbsoluteEncoder() {
    return swivel.getAbsoluteEncoder().getPosition();
  }

  public boolean isSwivelReadyToShoot() {
    return Math.abs(getSwivelEncoder() - swivelSetpoint) <= 1;
  }

  private void privSetHood(double speed) {
    turretHood.set(speed);
  }

  public double getHoodSetpoint() {
    return turretHoodSetpoint;
  }

  public void setHoodSetpoint(double setpoint) {
    // if (setpoint != swivelSetpoint) {
    //   swivelPID.reset(getSwivelEncoder());
    // }

    turretHoodSetpoint = setpoint;

    double calculation = 0;
    hoodPID.setGoal(setpoint);
    // double pidValue = hoodPID.calculate(getHoodEncoder());

    if (!hoodPID.atGoal()) {
      calculation = hoodPID.calculate(getHoodEncoder());
    } else {
      hoodPID.reset(getHoodEncoder());
    }

    // if (!hoodPID.atSetpoint()) {
    //   privSetHood(MathUtil.clamp(calculation, -1, 1));
    // }

    privSetHood(MathUtil.clamp(calculation, -1, 1));

    SmartDashboard.putNumber("Hood Speed", calculation);
  }

  public double getHoodEncoder() {
    return turretHood.getPosition().getValueAsDouble();
  }

  public boolean isHoodReadyToShoot() {
    return Math.abs(getHoodEncoder() - turretHoodSetpoint) <= 1;
  }

  public void setSpindexer(double speed){
    spindexerMaster.set(speed);
  }
  
  public static boolean getIsShooting(){
    return isShooting;
  }

  public static void toggleIsShooting(){
    isShooting = !isShooting;
  }

  public void addInstruments(Orchestra orchestra){
    // orchestra.addInstrument(swivel);
    // orchestra.addInstrument(flywheelFollower);
    // orchestra.addInstrument(flywheelMaster);
    // orchestra.addInstrument(spindexerFollower);
    // orchestra.addInstrument(spindexerFollower);
    // orchestra.addInstrument(turretHood);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Swivel Internal Encoder", getSwivelEncoder());
    SmartDashboard.putNumber("Swivel Absolute Encoder", getSwivelAbsoluteEncoder());
    SmartDashboard.putNumber("Swivel Setpoint", swivelSetpoint);
  }
}
