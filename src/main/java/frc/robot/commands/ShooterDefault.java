package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterDefault extends Command {
    
    private ShooterSubsystem s_ShooterSubsystem;

    private PIDController pid;

    private Pose2d botPose;

    public ShooterDefault() {

        s_ShooterSubsystem = ShooterSubsystem.getInstance();

        pid = new PIDController(0, 0, 0);

    }

    @Override
    public void initialize() {



    }

    @Override
    public void execute() {

        botPose = new Pose2d(); //TODO: Replace with estimated robot odometry plus trig for turret position

        // Distance in x and y axis respectively
        double dx = botPose.getX() - Constants.hubPosition.getX();
        double dy = botPose.getY() - Constants.hubPosition.getY();

        // angle in radians of the theoretical setpoint while stood still.
        double theta = Math.atan2(dy, dx);

        // distance away from center point of the turret to the center of the hub
        double hypotenuse = Math.hypot(dx, dy);

        // hypothetically, this math should give the launcher angle in degrees from 75 to 85 scaled to distance away from the center of the hub
        double launchAngle = ((hypotenuse - Constants.minimumHubDist) / (Constants.maximumHubDist - Constants.minimumHubDist)) * (Constants.maximumTurretAngle - Constants.minimumTurretAngle) + Constants.minimumTurretAngle;

        // Distance the ball needs to hit for the ball to hit the height and position of the hub along it's parabola
        double shootingTargetDistance = hypotenuse + (Constants.hubPosition.getZ() / Math.tan(launchAngle));

        // Initial velocity in m/s that the ball should have to travel to score (9.81 is gravity)
        double vO = Math.sqrt((shootingTargetDistance * 9.81) / Math.sin(2 * launchAngle));

        // TODO: Figure out velocity to motor speed scale irl (and if it's linear like this or not)
        double motorSpeed = vO / Constants.maximumBallSpeed;

        

    }


}
