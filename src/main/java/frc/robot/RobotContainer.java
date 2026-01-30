// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ShooterDefault;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private Swerve s_Swerve = Swerve.getInstance();
  private ShooterSubsystem s_ShooterSubsystem = ShooterSubsystem.getInstance();

  private final Joystick driver = new Joystick(0);
  private final Joystick operator = new Joystick(1);

  private final JoystickButton driver_A_Function = new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton driver_B_Function = new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton driver_X_Function = new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton driver_Y_Function = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton driver_Start_Function = new JoystickButton(driver,
      XboxController.Button.kStart.value);
  private final JoystickButton driver_Back_Function = new JoystickButton(driver, XboxController.Button.kBack.value);
  private final JoystickButton driver_LeftBumper_Function = new JoystickButton(driver,
      XboxController.Button.kLeftBumper.value);
  private final JoystickButton driver_RightBumper_Function = new JoystickButton(driver,
      XboxController.Button.kRightBumper.value);
  private final POVButton driverLeftDpad = new POVButton(driver, 270);
  private final POVButton driverRightDpad = new POVButton(driver, 90);
  private final POVButton driverUpDpad = new POVButton(driver, 0);
  private final POVButton driverDownDpad = new POVButton(driver, 180);
  private final JoystickButton driverStartButton = new JoystickButton(driver, XboxController.Button.kStart.value);


  private final JoystickButton operatorX = new JoystickButton(operator, XboxController.Button.kX.value);
  private final JoystickButton operatorY = new JoystickButton(operator, XboxController.Button.kY.value);
  private final JoystickButton operatorA = new JoystickButton(operator, XboxController.Button.kA.value);
  private final JoystickButton operatorB = new JoystickButton(operator, XboxController.Button.kB.value);
  private final JoystickButton operatorStart = new JoystickButton(operator, XboxController.Button.kStart.value);
  private final JoystickButton operatorLeftStickDown = new JoystickButton(operator,
      XboxController.Button.kLeftStick.value);
  private final JoystickButton operatorRightStickDown = new JoystickButton(operator,
      XboxController.Button.kRightStick.value);

  private final JoystickButton operatorLeftBumper = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
  private final JoystickButton operatorRightBumper = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
  private final POVButton operatorUpDPad = new POVButton(operator, 0);
  private final POVButton operatorRightDPad = new POVButton(operator, 90);
  private final POVButton operatorDownDPad = new POVButton(operator, 180);
  private final POVButton operatorLeftDPad = new POVButton(operator, 270);

  
  // The robot's subsystems and commands are defined here...
  // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandXboxController m_driverController = FIXED FOR ERRORS
  //     new CommandXboxController(OperatorConstants.kDriverControllerPort); FIXED FOR ERRORS

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    s_Swerve.setDefaultCommand(new TeleopDrive(driver, operator));
    s_ShooterSubsystem.setDefaultCommand(new ShooterDefault());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new Command() {};
  }
}
