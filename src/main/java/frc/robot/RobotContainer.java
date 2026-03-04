// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoAlignClimb;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeExtensionCommand;
import frc.robot.commands.L1Climb;
import frc.robot.commands.ShooterDefault;
import frc.robot.commands.SpindexerCommand;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.ClimbDefault;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CompressorSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

  private ChoreoManager s_choreoManager;
  private LimeLightSubsystem limelightLeft;
  private LimeLightSubsystem limelightSky;
  private LimeLightSubsystem limelightRight;

  private Swerve s_Swerve = Swerve.getInstance();
  private ShooterSubsystem s_ShooterSubsystem = ShooterSubsystem.getInstance();
  private ClimbSubsystem s_ClimbSubsystem = ClimbSubsystem.getInstance();
  private CompressorSubsystem s_CompressorSubsystem = CompressorSubsystem.getInstance();

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

    public AutoChooser autoChooser;

  // The robot's subsystems and commands are defined here...
  // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandXboxController m_driverController = FIXED FOR ERRORS
  //     new CommandXboxController(OperatorConstants.kDriverControllerPort); FIXED FOR ERRORS

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_Swerve = Swerve.getInstance();
    s_choreoManager = ChoreoManager.getInstance();
    limelightLeft = LimeLightSubsystem.getInstance(Constants.limelightLeft);
    limelightSky = LimeLightSubsystem.getInstance(Constants.limelightSky);
    limelightRight = LimeLightSubsystem.getInstance(Constants.limelightRight);
    // Configure the trigger bindings
    configureBindings();


        // Create the auto chooser
    autoChooser = new AutoChooser();

    // Add options to the chooser
    // autoChooser.addRoutine("Middle Auto", s_choreoSubsystem::onePieceAuto);
    autoChooser.addRoutine("SweepAuto No Climb", () -> s_choreoManager.sweepAuto(false, false));
    autoChooser.addRoutine("SweepAuto Left Climb", () -> s_choreoManager.sweepAuto(true, false));
    autoChooser.addRoutine("SweepAuto Right Climb", () -> s_choreoManager.sweepAuto(true, true));

    autoChooser.addRoutine("RightAuto No Climb", () -> s_choreoManager.rightHalfAuto(false, false));
    autoChooser.addRoutine("RightAuto Left Climb", () -> s_choreoManager.rightHalfAuto(true, false));
    autoChooser.addRoutine("RightAuto Right Climb", () -> s_choreoManager.rightHalfAuto(true, true));

    autoChooser.addRoutine("LeftAuto No Climb", () -> s_choreoManager.leftAuto(false, false));
    autoChooser.addRoutine("LeftAuto Left Climb", () -> s_choreoManager.leftAuto(true, false));
    autoChooser.addRoutine("LeftAuto Right Climb", () -> s_choreoManager.leftAuto(true, true));

    autoChooser.addRoutine("NonNeutral No Climb", () -> s_choreoManager.nonNeutralAuto(false, false));
    autoChooser.addRoutine("NonNeutral Left Climb", () -> s_choreoManager.nonNeutralAuto(true, false));
    autoChooser.addRoutine("NonNeutral Right Climb", () -> s_choreoManager.nonNeutralAuto(true, true));

    autoChooser.addRoutine("LeftHalfTwice No Climb", () -> s_choreoManager.leftHalfTwice(false, false));
    autoChooser.addRoutine("LeftHalfTwice Left Climb", () -> s_choreoManager.leftHalfTwice(true, false));
    autoChooser.addRoutine("LeftHalfTwice Right Climb", () -> s_choreoManager.leftHalfTwice(true, true));

    autoChooser.addRoutine("RightDoubleAuto No Climb", () -> s_choreoManager.rightDoubleAuto(false, false));
    autoChooser.addRoutine("RightDoubleAuto Left Climb", () -> s_choreoManager.rightDoubleAuto(true, false));
    autoChooser.addRoutine("RightDoubleAuto Right Climb", () -> s_choreoManager.rightDoubleAuto(true, true));
    //autoChooser.addRoutine("L2 IJ", s_choreoSubsystem::twoPieceIJAutoL2); //If we need an L2 Auto

    // autoChooser.addCmd("Example Auto Command", this::exampleAutoCommand);

    // Put the auto chooser on the dashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
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
    s_ShooterSubsystem.setDefaultCommand(new ShooterDefault(operator));
    s_ClimbSubsystem.setDefaultCommand(new ClimbDefault(operator));
    operatorA.whileTrue(new SpindexerCommand(true));

    driver_A_Function.onTrue(new InstantCommand(() -> s_ShooterSubsystem.toggleIsShooting()));
    driver_RightBumper_Function.whileTrue(new IntakeCommand(true));
    driver_Start_Function.onTrue(new IntakeExtensionCommand());
    // driverLeftDpad.whileTrue(new AutoAlignClimb(AlignPosition.LeftOffset, 0));
    // driverRightDpad.whileTrue(new AutoAlignClimb(AlignPosition.RightOffset, 0));
    // driver_Y_Function.whileTrue(new L1Climb());
  }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
  }
}
