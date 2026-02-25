// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
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
  private TalonFX kicker;
  //TODO: Add kicker motor

  public double swivelSetpoint = 0;
  public double turretHoodSetpoint = 0;

  private PIDController swivelPID;
  private PIDController hoodPID;
  
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
    // swivel = new TalonFX(0);
    spindexerMaster = new TalonFX(21);
    spindexerFollower = new TalonFX(22);
    // turretHood = new TalonFX(0);
    // kicker = new TalonFX(0);
    //Change configs as need be
    
    // flywheelFollower.setControl(new Follower(flywheelMaster.getDeviceID(), MotorAlignmentValue.Aligned));
    spindexerFollower.setControl(new Follower(spindexerMaster.getDeviceID(), MotorAlignmentValue.Aligned));

    swivelPID = new PIDController(0, 0, 0);
    hoodPID = new PIDController(0, 0, 0);

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
  public double getHoodSetpoint() {
    return turretHoodSetpoint;
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
}
