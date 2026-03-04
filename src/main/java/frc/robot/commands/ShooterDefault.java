package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve;

public class ShooterDefault extends Command {
    
    private ShooterSubsystem s_ShooterSubsystem;

    private Swerve s_Swerve;

    private Pose2d botPose;

    private Field2d fieldTest;


    private Joystick operator;
    private Pose3d targetPose;

    private int delayCounter = 0;


    private boolean isManual = false;

    public ShooterDefault(Joystick operator) {
        s_ShooterSubsystem = ShooterSubsystem.getInstance();
        this.operator = operator;
        s_Swerve = Swerve.getInstance();    

        SmartDashboard.putNumber("TestX", 0);
        SmartDashboard.putNumber("TestY", 0);
        SmartDashboard.putNumber("TestDegrees", 0);

        fieldTest = new Field2d();

        addRequirements(s_ShooterSubsystem);
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


    private void attemptToShoot(double motorSpeed){
        if (s_ShooterSubsystem.getIsShooting()) {
            s_ShooterSubsystem.setFlywheels(motorSpeed);

            if (delayCounter >= 25){ //0.5 second delay
                s_ShooterSubsystem.setKicker(1);
                if (delayCounter >= 50){ //another 0.5 sec delay
                    s_ShooterSubsystem.setSpindexer(0.1);
                }
            }
            delayCounter++;
        } else {
            s_ShooterSubsystem.setFlywheels(0);
            s_ShooterSubsystem.setKicker(0);
            s_ShooterSubsystem.setSpindexer(0);
            delayCounter = 0;
        }
    }

    @Override
    public void execute() {

        if (operator.getRawButtonPressed(XboxController.Button.kRightStick.value)) {
            isManual = !isManual;
        }

        if (isManual) {
            double manualSwivel = 1 * MathUtil.applyDeadband(operator.getRawAxis(XboxController.Axis.kRightX.value), Constants.stickDeadband);
            double manualHood = 5 * MathUtil.applyDeadband(operator.getRawAxis(XboxController.Axis.kRightY.value), Constants.stickDeadband);
            s_ShooterSubsystem.setSwivelSetpoint(s_ShooterSubsystem.getSwivelSetpoint() - manualSwivel);
            s_ShooterSubsystem.setHoodSetpoint(s_ShooterSubsystem.getHoodSetpoint() - manualHood);
            if (s_ShooterSubsystem.getIsShooting()) { 
                s_ShooterSubsystem.setFlywheels(-1);
                s_ShooterSubsystem.setKicker(.2);
            } else {
                s_ShooterSubsystem.setFlywheels(0);
                s_ShooterSubsystem.setKicker(0);
            }
            return;
        }

        // botPose = new Pose2d(SmartDashboard.getNumber("TestX", 0), SmartDashboard.getNumber("TestY", 0), s_Swerve.getEstimatedPosition().getRotation()); //SmartDashboard.getNumber("TestDegrees", 0)
        botPose = s_Swerve.getEstimatedPosition(); // TODO: Uncomment this line once Swerve is merged into main!!!
        SmartDashboard.putNumber("EstimatorRot", s_Swerve.getEstimatedPosition().getRotation().getDegrees());

        fieldTest.setRobotPose(botPose);
        SmartDashboard.putData("TestField", fieldTest);

        if (s_Swerve.isInNeutral(botPose)){
            targetPose = getShuttleTargetPose();
        }
        else{
            targetPose = getHubTargetPose();
        }
        // targetPose = getHubTargetPose();

        double turretDist = Math.sqrt((Constants.turretPoseRobotReletive.getY() * Constants.turretPoseRobotReletive.getY()) + (Constants.turretPoseRobotReletive.getX() * Constants.turretPoseRobotReletive.getX()));

        // double turretTheta = Math.atan2(Constants.turretPoseRobotReletive.getY(), Constants.turretPoseRobotReletive.getX()) + botPose.getRotation().getRadians() - (Math.PI/2);
        double turretTheta = Rotation2d.fromRadians(Math.atan2(Constants.turretPoseRobotReletive.getY(), Constants.turretPoseRobotReletive.getX())).minus(Rotation2d.fromRadians((Math.PI/2))).getRadians();
        SmartDashboard.putNumber("TurretTheta", turretTheta * (180/Math.PI));

        double botX = botPose.getX() + Math.cos(turretTheta) * turretDist;
        double botY = botPose.getY() + Math.sin(turretTheta) * turretDist;

        Pose2d turretPoseFieldRelative = new Pose2d(botX, botY, Rotation2d.fromDegrees(botPose.getRotation().getDegrees() + Constants.turretPoseRobotReletive.getRotation().getDegrees()));
        SmartDashboard.putString("Turret Pose", turretPoseFieldRelative.toString());

        // Distance in x and y axis respectively
        double dx = targetPose.getX() - turretPoseFieldRelative.getX();
        double dy = targetPose.getY() - turretPoseFieldRelative.getY();

        SmartDashboard.putNumber("dX", dx);
        SmartDashboard.putNumber("dY", dy);
        // angle in radians of the theoretical setpoint while stood still.
        double thetaDegrees = Math.toDegrees(Math.atan2(dy, dx));

        SmartDashboard.putNumber("ThetaDegrees", thetaDegrees);

        // distance away from center point of the turret to the center of the hub
        double hypotenuse = Math.hypot(dx, dy);
        SmartDashboard.putNumber("Distance From Hub", hypotenuse);

        // hypothetically, this math should give the launcher angle in degrees from 75 to 85 scaled to distance away from the center of the hub
        double launchAngleDegrees = ((hypotenuse - Constants.minimumHubDist) / (Constants.maximumHubDist - Constants.minimumHubDist)) * (Constants.maximumHoodAngle - Constants.minimumHoodAngle) + Constants.minimumHoodAngle;
        SmartDashboard.putNumber("Launch Angle", launchAngleDegrees);

        // Distance the ball needs to hit for the ball to hit the height and position of the hub along it's parabola
        double shootingTargetDistance = hypotenuse + (targetPose.getZ() / Math.tan(launchAngleDegrees));

        // Initial velocity in m/s that the ball should have to travel to score (9.81 is gravity)
        double vO = Math.sqrt((shootingTargetDistance * 9.81) / Math.sin(2 * Math.toRadians(launchAngleDegrees)));

        // TODO: Figure out velocity to motor speed scale irl (and if it's linear like this or not)
        double motorSpeed = vO / Constants.maximumBallSpeed;

        // get the theta angle relative to robot rotation converted to encoder values
        double robotRelativeAngleDegrees = thetaDegrees - botPose.getRotation().getDegrees() + Constants.turretPoseRobotReletive.getRotation().getDegrees();

        double robotRelativeSwivelEncoder = robotRelativeAngleDegrees * Constants.swivelEncoderPerDegrees;


        
        if (thetaDegrees <= -90 && thetaDegrees >= -120) {
            s_ShooterSubsystem.setSwivelSetpoint(0);
        }
        else if (thetaDegrees >= 90 && thetaDegrees <= 120){
            s_ShooterSubsystem.setSwivelSetpoint(Constants.maximumSwivelEncoder);
        }
        else {
            s_ShooterSubsystem.setSwivelSetpoint(robotRelativeSwivelEncoder);
        }

        s_ShooterSubsystem.setHoodSetpoint(launchAngleDegrees / Constants.hoodEncoderPerDegree);


        // if (s_ShooterSubsystem.isSwivelReadyToShoot() && s_ShooterSubsystem.isHoodReadyToShoot()) { 
            // attemptToShoot(motorSpeed);

    }


}