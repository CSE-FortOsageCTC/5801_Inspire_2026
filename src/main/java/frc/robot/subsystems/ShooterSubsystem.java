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
  private SparkMax hood;
  private TalonFX kicker;
  //TODO: Add kicker motor

  public double swivelSetpoint = 0;
  public double hoodSetpoint = 0;

  public double highSpindexterCurrentCounter = 0;
  public double unjammingCounter = 0;
  public boolean isJammed = false;

  private ProfiledPIDController swivelPID;
  private ProfiledPIDController hoodPID;
  
  private boolean isShooting = false;
 
  public static ShooterSubsystem getInstance(){
        if (shooterSubsystem == null){
            shooterSubsystem = new ShooterSubsystem();
        } 
        return (shooterSubsystem);
    }
  private ShooterSubsystem() {
    //TODO assign IDs 
    flywheelMaster = new TalonFX(30);
    flywheelFollower = new TalonFX(31);
    swivel = new SparkMax(55, MotorType.kBrushless);
    spindexerMaster = new TalonFX(22);
    // spindexerFollower = new TalonFX(22);
    hood = new SparkMax(56, MotorType.kBrushless);
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
    hood.getEncoder().setPosition(0);

    flywheelFollower.setControl(new Follower(flywheelMaster.getDeviceID(), MotorAlignmentValue.Opposed));
    // spindexerFollower.setControl(new Follower(spindexerMaster.getDeviceID(), MotorAlignmentValue.Aligned));

    swivelPID = new ProfiledPIDController(0.04, 0.001, 0.001, new TrapezoidProfile.Constraints(500, 500));
    swivelPID.setTolerance(0.75);

    hoodPID = new ProfiledPIDController(0.07, 0, 0, new TrapezoidProfile.Constraints(0, 0));
    hoodPID.setTolerance(0.5);

    swivelSetpoint = getSwivelEncoder();
    hoodSetpoint = getHoodEncoder();

    swivelPID.reset(swivelSetpoint);
    hoodPID.reset(hoodSetpoint);

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

    swivelSetpoint = MathUtil.clamp(setpoint, Constants.minimumSwivelEncoder, Constants.maximumSwivelEncoder);

    double calculation = 0;
    swivelPID.setGoal(swivelSetpoint);
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
    hood.set(speed);
  }

  public double getHoodSetpoint() {
    return hoodSetpoint;
  }

  public void setHoodSetpoint(double setpoint) {
    // if (setpoint != swivelSetpoint) {
    //   swivelPID.reset(getSwivelEncoder());
    // }

    
    hoodSetpoint = MathUtil.clamp(setpoint, Constants.minimumHoodEncoder, Constants.maximumHoodEncoder);

    double calculation = 0;
    hoodPID.setGoal(hoodSetpoint);
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
    return hood.getEncoder().getPosition();
  }

  public boolean isHoodReadyToShoot() {
    return Math.abs(getHoodEncoder() - hoodSetpoint) <= 1;
  }

  public void setSpindexer(double speed){
    if (isJammed || unjammingCounter <= 25) {
      spindexerMaster.set(-0.15);
      unjammingCounter += 1;
    }
    else if (unjammingCounter >= 25) {
      unjammingCounter = 0;
    }
    else {
      spindexerMaster.set(speed);
    }
  }
  
  public boolean getIsShooting(){
    return isShooting;
  }

  public void toggleIsShooting(){
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
    SmartDashboard.putNumber("Swivel Setpoint", swivelSetpoint);

    SmartDashboard.putNumber("Hood Internal Encoder", getHoodEncoder());
    SmartDashboard.putNumber("Hood Setpoint", hoodSetpoint);


    if (Math.abs(spindexerMaster.getStatorCurrent().getValueAsDouble()) >= 300) {
      highSpindexterCurrentCounter += 1;
    }
    else {
      highSpindexterCurrentCounter = 0;
      isJammed = false;
    }

    if (highSpindexterCurrentCounter >= 25) {
      isJammed = true;
    }
  }
}
