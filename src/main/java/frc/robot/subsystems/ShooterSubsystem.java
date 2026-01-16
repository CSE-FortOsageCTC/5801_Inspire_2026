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


  
 
  public static ShooterSubsystem getInstance(){
        if (shooterSubsystem == null){
            shooterSubsystem = new ShooterSubsystem();
        } 
        return (shooterSubsystem);
    }
  public ShooterSubsystem() {
    //TODO assign IDs 
    flywheelMaster = new TalonFX(0);
    flywheelFollower = new TalonFX(0);
    swivel = new TalonFX(0);
    spindexerMaster = new TalonFX(0);
    spindexerFollower = new TalonFX(0);
    //Change configs as need be
    
    flywheelFollower.setControl(new Follower(flywheelMaster.getDeviceID(), MotorAlignmentValue.Aligned));
    spindexerFollower.setControl(new Follower(spindexerMaster.getDeviceID(), MotorAlignmentValue.Aligned));



  }

  public void setFlywheels(double speed){
    flywheelMaster.set(speed);
  }

  public void setSwivel(double speed){
    swivel.set(speed);

  }

  public double getSwivelEncoder(){
    return swivel.getPosition().getValueAsDouble();
  }

  public void setSpindexer(double speed){
    spindexerMaster.set(speed);
  }



    

    

}
