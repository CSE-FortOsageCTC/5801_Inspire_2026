package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.AlignPosition; Error
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.commands.AutoAlignClimb;
// import frc.robot.Constants.ArmPosition; Error
// import frc.robot.commands.AlignToApril; Error
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeExtensionCommand;
import frc.robot.commands.IntakeExtensionDefault;
import frc.robot.subsystems.IntakeExtensionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve;

import java.sql.Driver;
import java.util.Optional;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.*;

public class ChoreoManager {

    private Swerve s_Swerve;
    private ShooterSubsystem s_ShooterSubsystem;
    private IntakeExtensionSubsystem s_IntakeExtensionSubsystem;
    private IntakeSubsystem s_IntakeSubsystem;
    private AutoFactory autoFactory;

    private PIDController autoXPID = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
    private PIDController autoYPID = new PIDController(Constants.AutoConstants.kPYController, 0, 0);
    private PIDController autoThetaPID = new PIDController(Constants.AutoConstants.kPThetaController, 0, 0);

    private static ChoreoManager choreoManager;

    private Trajectory<SwerveSample> trajectory;

    public static ChoreoManager getInstance() {
        if (choreoManager == null) {
            choreoManager = new ChoreoManager();
        }
        return choreoManager;
    }

    private ChoreoManager() {

        s_Swerve = Swerve.getInstance();
        s_ShooterSubsystem = ShooterSubsystem.getInstance();
        s_IntakeExtensionSubsystem = IntakeExtensionSubsystem.getInstance();
        s_IntakeSubsystem = IntakeSubsystem.getInstance();

        autoFactory = new AutoFactory(
                this::getPose,
                this::setPose,
                this::autoDrive,
                isRed(),
                s_Swerve);

        // autoFactory.bind("ArmGround", new InstantCommand(() -> ArmPosition.setPosition(ArmPosition.Ground)));
        autoFactory.bind("Intake", new InstantCommand(() -> s_IntakeSubsystem.setIntakeSpeed(0.35)));
        autoFactory.bind("IntakeEnd", new InstantCommand(() -> s_IntakeSubsystem.setIntakeSpeed(0)));
        
    }

    private void switchPipelines(int pipeline) {

        LimeLightSubsystem.getLeftInstance().setPipeline(pipeline);
        LimeLightSubsystem.getRightInstance().setPipeline(pipeline);
    }

    private boolean isRed() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        System.out.println(alliance);
        return alliance.isPresent() && alliance.get().equals(Alliance.Red);
    }

    private Pose2d getPose() {
        // s_Swerve.setPose(trajectory.getInitialPose());
        // System.out.println(s_Swerve.getPose().getRotation().toString());
        return s_Swerve.getPose();
    }

    private void setPose(Pose2d pose) {
        s_Swerve.setPose(pose);
    }

    private void autoDrive(SwerveSample sample) {
        s_Swerve.autoDrive(sample);
    }

    public Command setupAutonomousChoreoPath(String traj) {

        return autoFactory.newRoutine(traj).cmd();

    }

    // MARK: Pre-Load Auto
    public AutoRoutine preLoadAuto(boolean willClimb, boolean isRightClimb) {
        // System.out.println("this is before the auto routine");
        AutoRoutine routine = autoFactory.newRoutine("PreLoadAuto");

        // System.out.println("this is the top of the auto code");

        // Load the routine's trajectories
        AutoTrajectory traj_PreLoadAuto = routine.trajectory("PreLoadAuto");
        AutoTrajectory traj_HubToOutpost = routine.trajectory("HubToOutpost");
        // AlignPosition alignPosition = isRightClimb ? AlignPosition.RightOffset : AlignPosition.LeftOffset;
        // if (!willClimb)
        // {
        //     alignPosition = AlignPosition.NoPos;
        // }
        // When the routine begins, reset odometry and start the first trajectory
        routine.active().onTrue(
            Commands.sequence(
                traj_PreLoadAuto.resetOdometry(),
                // new InstantCommand(%() ->
                // ArmPosition.setPosition(ArmPosition.StartingConfig)),
                new InstantCommand(() -> s_Swerve.setHeading(Rotation2d.fromDegrees(0))),
                traj_PreLoadAuto.cmd().withTimeout(10),
                new InstantCommand(() -> s_Swerve.drive(new Translation2d(0, 0), 0, true, true)),
                new IntakeExtensionCommand(),
                new InstantCommand(() -> s_ShooterSubsystem.toggleIsShooting()),
                new WaitCommand(5),
                new InstantCommand(() -> s_ShooterSubsystem.toggleIsShooting()),
                traj_HubToOutpost.cmd().withTimeout(10),
                new InstantCommand(() -> s_Swerve.drive(new Translation2d(0, 0), 0, true, true)),
                new InstantCommand(() -> s_ShooterSubsystem.toggleIsShooting()),
                new WaitCommand(5),
                new InstantCommand(() -> s_IntakeExtensionSubsystem.setIsJiggling(true)),
                new IntakeCommand(true)
                
                // new AutoAlignClimb(alignPosition, 0)

        ));
        return routine;
    }

    // MARK: Left Sweep Auto
    public AutoRoutine leftSweepAuto(boolean willClimb, boolean isRightClimb) {
        // System.out.println("this is before the auto routine");
        AutoRoutine routine = autoFactory.newRoutine("LeftSweepAuto");

        // System.out.println("this is the top of the auto code");

        // Load the routine's trajectories
        AutoTrajectory traj_LeftSweep = routine.trajectory("LeftSweep");
        // AlignPosition alignPosition = isRightClimb ? AlignPosition.RightOffset : AlignPosition.LeftOffset;
        // if (!willClimb)
        // {
        //     alignPosition = AlignPosition.NoPos;
        // }

        

        // When the routine begins, reset odometry and start the first trajectory
        routine.active().onTrue(
            Commands.sequence(
                traj_LeftSweep.resetOdometry(),
                // new InstantCommand(%() ->
                // ArmPosition.setPosition(ArmPosition.StartingConfig)),
                new InstantCommand(() -> {s_ShooterSubsystem.autoFlywheelModifier = 1.04;}),
                new IntakeExtensionCommand(),
                new InstantCommand(() -> s_Swerve.setHeading(Rotation2d.fromDegrees(0))),
                traj_LeftSweep.cmd(),
                new InstantCommand(() -> s_Swerve.drive(new Translation2d(0, 0), 0, true, true)),
                new InstantCommand(() -> s_ShooterSubsystem.toggleIsShooting()),
                new WaitCommand(2),
                new InstantCommand(() -> s_IntakeExtensionSubsystem.setIsJiggling(true)),
                new WaitCommand(2),
                new InstantCommand(() -> s_ShooterSubsystem.toggleIsShooting()),
                new InstantCommand(() -> s_IntakeExtensionSubsystem.setIsJiggling(false)),
                traj_LeftSweep.cmd(),
                new InstantCommand(() -> s_Swerve.drive(new Translation2d(0, 0), 0, true, true)),
                new InstantCommand(() -> s_ShooterSubsystem.toggleIsShooting()),
                new WaitCommand(2),
                new InstantCommand(() -> s_IntakeExtensionSubsystem.setIsJiggling(true)),
                new IntakeCommand(true)
                
                // new AutoAlignClimb(alignPosition, 0)

        ));
        
        return routine;
    }

    // MARK: Right Sweep Auto
    public AutoRoutine rightSweepAuto(boolean willClimb, boolean isRightClimb) {
        // System.out.println("this is before the auto routine");
        AutoRoutine routine = autoFactory.newRoutine("RightSweepAuto");

        // System.out.println("this is the top of the auto code");

        // Load the routine's trajectories
        AutoTrajectory traj_LeftSweep = routine.trajectory("LeftSweep").mirrorY();
        // AlignPosition alignPosition = isRightClimb ? AlignPosition.RightOffset : AlignPosition.LeftOffset;
        // if (!willClimb)
        // {
        //     alignPosition = AlignPosition.NoPos;
        // }

        

        // When the routine begins, reset odometry and start the first trajectory
        routine.active().onTrue(
            Commands.sequence(
                traj_LeftSweep.resetOdometry(),
                // new InstantCommand(%() ->
                // ArmPosition.setPosition(ArmPosition.StartingConfig)),
                new InstantCommand(() -> {s_ShooterSubsystem.autoFlywheelModifier = 1.02;}),
                new IntakeExtensionCommand(),
                new InstantCommand(() -> s_Swerve.setHeading(Rotation2d.fromDegrees(0))),
                traj_LeftSweep.cmd(),
                new InstantCommand(() -> s_Swerve.drive(new Translation2d(0, 0), 0, true, true)),
                new InstantCommand(() -> s_ShooterSubsystem.toggleIsShooting()),
                new WaitCommand(2),
                new InstantCommand(() -> s_IntakeExtensionSubsystem.setIsJiggling(true)),
                new WaitCommand(2),
                new InstantCommand(() -> s_ShooterSubsystem.toggleIsShooting()),
                new InstantCommand(() -> s_IntakeExtensionSubsystem.setIsJiggling(false)),
                traj_LeftSweep.cmd(),
                new InstantCommand(() -> s_Swerve.drive(new Translation2d(0, 0), 0, true, true)),
                new InstantCommand(() -> s_ShooterSubsystem.toggleIsShooting()),
                new WaitCommand(2),
                new InstantCommand(() -> s_IntakeExtensionSubsystem.setIsJiggling(true)),
                new IntakeCommand(true)
                
                // new AutoAlignClimb(alignPosition, 0)

        ));
        
        return routine;
    }

}