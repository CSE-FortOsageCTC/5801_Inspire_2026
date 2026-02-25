package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve;

public class ShooterDefault extends Command {
    
    private ShooterSubsystem s_ShooterSubsystem;

    private Swerve s_Swerve;

    private Pose2d botPose;


    private Joystick operator;
    private Pose3d targetPose;



    private boolean isManual = false;

    public ShooterDefault(Joystick operator) {
        s_ShooterSubsystem = ShooterSubsystem.getInstance();
        this.operator = operator;
        s_Swerve = Swerve.getInstance();    
    }

    private boolean isNorth(Pose2d botPose){
        if (botPose.getY() >= 4.021328){
            return true;
        }
        return false;
    }

    private Pose3d getShuttleTargetPose(){
        if (DriverStation.Alliance.Red.equals(DriverStation.getAlliance().get())) {
            if (isNorth(botPose)){
                return Constants.redNorthShuttleTarget;
            }
            else{
                return Constants.redSouthShuttleTarget;
            }
        }

        else{
            if (isNorth(botPose)){
                return Constants.blueNorthShuttleTarget;
            }
            else{
                return Constants.blueSouthShuttleTarget;
            }
        }
    }

    private Pose3d getHubTargetPose(){
        if (DriverStation.Alliance.Red.equals(DriverStation.getAlliance().get())) {
            return Constants.redHubPosition;
        }
        else{
            return Constants.blueHubPosition;
        }
    }

    @Override
    public void execute() {

        if (operator.getRawButton(XboxController.Button.kRightStick.value)) {
            isManual = !isManual;
        }

        if (isManual) {
            double manualSwivel = operator.getRawAxis(XboxController.Axis.kRightX.value);
            double manualHood = operator.getRawAxis(XboxController.Axis.kRightY.value);
            s_ShooterSubsystem.setSwivelSetpoint(s_ShooterSubsystem.getSwivelSetpoint() + manualSwivel);
            s_ShooterSubsystem.setHoodSetpoint(s_ShooterSubsystem.getHoodSetpoint() + manualHood);
            if (ShooterSubsystem.getIsShooting()) { 
                s_ShooterSubsystem.setFlywheels(1);
                s_ShooterSubsystem.setKicker(1);
            } else {
                s_ShooterSubsystem.setFlywheels(0);
                s_ShooterSubsystem.setKicker(0);
            }
            return;
        }

        botPose = new Pose2d();
        // botPose = swerveEstimator.getEstimatedPosition(); // TODO: Uncomment this line once Swerve is merged into main!!!

        if (s_Swerve.isInNeutral()){
            targetPose = getShuttleTargetPose();
        }
        else{
            targetPose = getHubTargetPose();
        }

        Pose2d turretPoseFieldRelative = new Pose2d(botPose.getX() + (Math.sin(botPose.getRotation().getRadians()) * Constants.turretPoseRobotReletive.getX()), botPose.getY() + (Math.cos(botPose.getRotation().getRadians()) * Constants.turretPoseRobotReletive.getY()), Rotation2d.fromDegrees(botPose.getRotation().getDegrees() + Constants.turretPoseRobotReletive.getRotation().getDegrees()));

        // Distance in x and y axis respectively
        double dx = turretPoseFieldRelative.getX() - targetPose.getX();
        double dy = turretPoseFieldRelative.getY() - targetPose.getY();

        // angle in radians of the theoretical setpoint while stood still.
        double thetaDegrees = Math.toDegrees(Math.atan2(dy, dx));

        // distance away from center point of the turret to the center of the hub
        double hypotenuse = Math.hypot(dx, dy);

        // hypothetically, this math should give the launcher angle in degrees from 75 to 85 scaled to distance away from the center of the hub
        double launchAngleDegrees = ((hypotenuse - Constants.minimumHubDist) / (Constants.maximumHubDist - Constants.minimumHubDist)) * (Constants.maximumHoodAngle - Constants.minimumHoodAngle) + Constants.minimumHoodAngle;

        // Distance the ball needs to hit for the ball to hit the height and position of the hub along it's parabola
        double shootingTargetDistance = hypotenuse + (targetPose.getZ() / Math.tan(launchAngleDegrees));

        // Initial velocity in m/s that the ball should have to travel to score (9.81 is gravity)
        double vO = Math.sqrt((shootingTargetDistance * 9.81) / Math.sin(2 * Math.toRadians(launchAngleDegrees)));

        // TODO: Figure out velocity to motor speed scale irl (and if it's linear like this or not)
        double motorSpeed = vO / Constants.maximumBallSpeed;

        // get the theta angle relative to robot rotation converted to encoder values
        double robotRelativeAngleDegrees = thetaDegrees + botPose.getRotation().getDegrees() + Constants.turretPoseRobotReletive.getRotation().getDegrees();

        double robotRelativeSwivelEncoder = robotRelativeAngleDegrees * Constants.swivelEncoderPerDegrees;

        if (robotRelativeSwivelEncoder <= Constants.maximumSwivelEncoder && robotRelativeSwivelEncoder >= Constants.minimumSwivelEncoder) {
            s_ShooterSubsystem.setSwivelSetpoint(robotRelativeSwivelEncoder);
        }
   

        s_ShooterSubsystem.setHoodSetpoint(launchAngleDegrees);


        if (ShooterSubsystem.getIsShooting() && s_ShooterSubsystem.isSwivelReadyToShoot() && s_ShooterSubsystem.isHoodReadyToShoot()) { 
            s_ShooterSubsystem.setFlywheels(motorSpeed);
        } else {
            s_ShooterSubsystem.setFlywheels(0);
        }

    }


}
