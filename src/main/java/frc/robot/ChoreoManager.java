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
// import frc.robot.AlignPosition; Error
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
// import frc.robot.Constants.ArmPosition; Error
// import frc.robot.commands.AlignToApril; Error
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
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

        autoFactory = new AutoFactory(
                this::getPose,
                this::setPose,
                this::autoDrive,
                isRed(),
                s_Swerve);

        // autoFactory.bind("ArmGround", new InstantCommand(() -> ArmPosition.setPosition(ArmPosition.Ground)));
        autoFactory.bind("GroundIntake", new InstantCommand(() -> IntakeSubsystem.getInstance().setIntakeSpeed(-1)));
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

    // MARK: One Piece Auto
    public AutoRoutine onePieceAuto() {
        System.out.println("this is before the auto routine");
        AutoRoutine routine = autoFactory.newRoutine("onePiece");

        System.out.println("this is the top of the auto code");

        // Load the routine's trajectories
        AutoTrajectory traj_startToHG = routine.trajectory("startToHG");
        AutoTrajectory traj_ABToNet = routine.trajectory("ABToNet");
        AutoTrajectory traj_NettoIJ = routine.trajectory("NetToIJ");
        AutoTrajectory traj_IJtoNet = routine.trajectory("IJtoNet");
        AutoTrajectory traj_GetRP = routine.trajectory("GetRP");

        // When the routine begins, reset odometry and start the first trajectory
        routine.active().onTrue(
                Commands.sequence(
                        // traj_startToIJ.resetOdometry(),
                        // new InstantCommand(() ->
                        // ArmPosition.setPosition(ArmPosition.StartingConfig)),
                        new InstantCommand(() -> s_Swerve.setHeading(Rotation2d.fromDegrees(0))) // rotateBy(180);
                  
                )
            );

        return routine;
    }

    // MARK: Lollipop EF Pickup
    public AutoRoutine lollipopEFAutoPickup() {
        // System.out.println("this is before the auto routine");
        AutoRoutine routine = autoFactory.newRoutine("lollipopEF");

        // System.out.println("this is the top of the auto code");

        // Load the routine's trajectories
        AutoTrajectory traj_startToEF = routine.trajectory("startToEF");
        AutoTrajectory traj_EFto3 = routine.trajectory("EFtoThree");
        
        AutoTrajectory traj_3ToAB = routine.trajectory("threeToAB");
        AutoTrajectory traj_ABto2 = routine.trajectory("ABtoTwoEF");
        AutoTrajectory traj_2ToAB = routine.trajectory("twoToAB");

        // When the routine begins, reset odometry and start the first trajectory
        routine.active().onTrue(
            Commands.sequence(
                // traj_startToIJ.resetOdometry(),
                // new InstantCommand(() ->
                // ArmPosition.setPosition(ArmPosition.StartingConfig)),
                new InstantCommand(() -> s_Swerve.setHeading(Rotation2d.fromDegrees(0))),
                traj_startToEF.cmd().withTimeout(1),
                new InstantCommand(() -> s_Swerve.drive(new Translation2d(0, 0), 0, true, true))


        ));
        return routine;
    }

    // MARK: Lollipop IJ Pickup
    public AutoRoutine lollipopIJAutoPickup() {
        // System.out.println("this is before the auto routine");
        AutoRoutine routine = autoFactory.newRoutine("lollipopIJ");

        // System.out.println("this is the top of the auto code");

        // Load the routine's trajectories
        AutoTrajectory traj_startToIJ = routine.trajectory("startToIJ");
        AutoTrajectory traj_IJto1 = routine.trajectory("IJtoOne");
        
        AutoTrajectory traj_1ToAB = routine.trajectory("oneToAB");
        AutoTrajectory traj_ABto2 = routine.trajectory("ABtoTwo");
        AutoTrajectory traj_2ToAB = routine.trajectory("twoToAB");

        // When the routine begins, reset odometry and start the first trajectory
        routine.active().onTrue(
            Commands.sequence(
                // traj_startToIJ.resetOdometry(),
                // new InstantCommand(() ->
                // ArmPosition.setPosition(ArmPosition.StartingConfig)),
                new InstantCommand(() -> s_Swerve.setHeading(Rotation2d.fromDegrees(0))),
                traj_startToIJ.cmd().withTimeout(1),
                new InstantCommand(() -> s_Swerve.drive(new Translation2d(0, 0), 0, true, true))
                

        ));
        return routine;
    }

    public AutoRoutine demoCircle() {
        // System.out.println("this is before the auto routine");
        AutoRoutine routine = autoFactory.newRoutine("demo");

        // System.out.println("this is the top of the auto code");

        // Load the routine's trajectories
        AutoTrajectory traj_circle = routine.trajectory("DemoCircle");

        // When the routine begins, reset odometry and start the first trajectory
        routine.active().onTrue(
            Commands.sequence(
                new InstantCommand(() -> s_Swerve.setHeading(Rotation2d.fromDegrees(0))),
                traj_circle.cmd()
        ));
        return routine;
    }

}