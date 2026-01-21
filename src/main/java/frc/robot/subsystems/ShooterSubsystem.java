// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

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



  }

  public void setFlywheels(double speed){
    flywheelMaster.set(speed);
  }

  public void privSetSwivel(double speed) {
    swivel.set(speed);
  }

  public void setSwivelSetpoint(double setpoint) {
    swivelSetpoint = setpoint;
  }

  public double getSwivelEncoder() {
    return swivel.getPosition().getValueAsDouble();
  }

  public void privSetHood(double speed) {
    turretHood.set(speed);
  }

  public void setHoodSetpoint(double setpoint) {
    turretHoodSetpoint = setpoint;
  }

  public double getHoodEncoder() {
    return turretHood.getPosition().getValueAsDouble();
  }

  public void setSpindexer(double speed){
    spindexerMaster.set(speed);
  }

}
