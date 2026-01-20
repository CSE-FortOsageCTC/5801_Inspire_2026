package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;


public class ClimbSubsystem {
    public static TalonFX leftClimber;
    public static TalonFX rightClimber;

    public ClimbSubsystem(){
        // s_Swerve = Swerve.getInstance();
        // ledSubsystem = LEDSubsystem.getInstance();
        
        leftClimber = new TalonFX(0);
        rightClimber = new TalonFX(0);
  }
}
