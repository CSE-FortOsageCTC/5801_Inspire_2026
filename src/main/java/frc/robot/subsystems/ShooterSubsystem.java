// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private static ShooterSubsystem shooterSubsystem;
  
  private TalonFX swivel;
  private TalonFX flywheelMaster;
  private TalonFX flywheelFollower;
  private TalonFX spindexerMaster;
  private TalonFX spindexerFollower;
  private TalonFX turretHood;

  public double swivelSetpoint = 0;
  public double turretHoodSetpoint = 0;

  private PIDController swivelPID;
  private PIDController hoodPID;
  
 
  public static ShooterSubsystem getInstance(){
        if (shooterSubsystem == null){
            shooterSubsystem = new ShooterSubsystem();
        } 
        return (shooterSubsystem);
    }
  private ShooterSubsystem() {
    //TODO assign IDs 
    flywheelMaster = new TalonFX(0);
    flywheelFollower = new TalonFX(0);
    swivel = new TalonFX(0);
    spindexerMaster = new TalonFX(0);
    spindexerFollower = new TalonFX(0);
    turretHood = new TalonFX(0);
    //Change configs as need be
    
    flywheelFollower.setControl(new Follower(flywheelMaster.getDeviceID(), MotorAlignmentValue.Aligned));
    spindexerFollower.setControl(new Follower(spindexerMaster.getDeviceID(), MotorAlignmentValue.Aligned));

    swivelPID = new PIDController(0, 0, 0);
    hoodPID = new PIDController(0, 0, 0);

  }

  public void setFlywheels(double speed){
    flywheelMaster.set(speed);
  }

  private void privSetSwivel(double speed) {
    swivel.set(speed);
  }

  public void setSwivelSetpoint(double setpoint) {
    swivelSetpoint = setpoint;

    double calculation = swivelPID.calculate(getSwivelEncoder(), setpoint);
    privSetSwivel(MathUtil.clamp(calculation, -1, 1)); // TODO: adjust clamps as needed
  }

  public double getSwivelEncoder() {
    return swivel.getPosition().getValueAsDouble();
  }

  public boolean isSwivelReadyToShoot() {
    return Math.abs(getSwivelEncoder() - swivelSetpoint) <= 1;
  }

  private void privSetHood(double speed) {
    turretHood.set(speed);
  }

  public void setHoodSetpoint(double setpoint) {
    turretHoodSetpoint = setpoint;

    double calculation = hoodPID.calculate(getSwivelEncoder(), setpoint);
    privSetHood(MathUtil.clamp(calculation, -1, 1)); // TODO: adjust clamps as needed
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

}
