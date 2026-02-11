package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterDefault extends Command {
    
    private ShooterSubsystem s_ShooterSubsystem;

    private Pose2d botPose;

    private Joystick operator;

    private boolean isManual = false;

    public ShooterDefault(Joystick operator) {
        s_ShooterSubsystem = ShooterSubsystem.getInstance();
        this.operator = operator;
        
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
            } else {
                s_ShooterSubsystem.setFlywheels(0);
            }
            return;
        }

        botPose = new Pose2d();
        // botPose = swerveEstimator.getEstimatedPosition(); // TODO: Uncomment this line once Swerve is merged into main!!!

        Pose2d turretPoseFieldRelative = new Pose2d(botPose.getX() + (Math.sin(botPose.getRotation().getRadians()) * Constants.turretPoseRobotReletive.getX()), botPose.getY() + (Math.cos(botPose.getRotation().getRadians()) * Constants.turretPoseRobotReletive.getY()), Rotation2d.fromDegrees(botPose.getRotation().getDegrees() + Constants.turretPoseRobotReletive.getRotation().getDegrees()));

        // Distance in x and y axis respectively
        double dx = turretPoseFieldRelative.getX() - Constants.hubPosition.getX();
        double dy = turretPoseFieldRelative.getY() - Constants.hubPosition.getY();

        // angle in radians of the theoretical setpoint while stood still.
        double thetaDegrees = Math.toDegrees(Math.atan2(dy, dx));

        // distance away from center point of the turret to the center of the hub
        double hypotenuse = Math.hypot(dx, dy);

        // hypothetically, this math should give the launcher angle in degrees from 75 to 85 scaled to distance away from the center of the hub
        double launchAngleDegrees = ((hypotenuse - Constants.minimumHubDist) / (Constants.maximumHubDist - Constants.minimumHubDist)) * (Constants.maximumHoodAngle - Constants.minimumHoodAngle) + Constants.minimumHoodAngle;

        // Distance the ball needs to hit for the ball to hit the height and position of the hub along it's parabola
        double shootingTargetDistance = hypotenuse + (Constants.hubPosition.getZ() / Math.tan(launchAngleDegrees));

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
