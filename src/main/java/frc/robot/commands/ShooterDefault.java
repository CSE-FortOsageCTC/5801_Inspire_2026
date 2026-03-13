package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.TurretState;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve;

public class ShooterDefault extends Command {
    
    private ShooterSubsystem s_ShooterSubsystem;
    private LEDSubsystem ledSubsystem;

    private Swerve s_Swerve;

    private Pose2d botPose;

    private Field2d fieldTest;

    private InterpolatingDoubleTreeMap shooterMap;

    private Joystick operator;
    private Pose3d targetPose;

    private boolean isUnderTrench = false;

    private boolean isWithinAutoTurn = false;

    private boolean isManual = true;

    public ShooterDefault(Joystick operator) {
        s_ShooterSubsystem = ShooterSubsystem.getInstance();
        this.operator = operator;
        s_Swerve = Swerve.getInstance();
        ledSubsystem = LEDSubsystem.getInstance();

        // SmartDashboard.putNumber("TestX", 0);
        // SmartDashboard.putNumber("TestY", 0);
        // SmartDashboard.putNumber("TestDegrees", 0);
        SmartDashboard.putNumber("TurretX", 0);
        SmartDashboard.putNumber("TurretY", 0);
        SmartDashboard.putNumber("FlywheelSpeed", 0);

        shooterMap = new InterpolatingDoubleTreeMap();

        shooterMap.put(0.0, 0.0); // 0 meters
        shooterMap.put(3.172, -0.35); // 2 meters
        shooterMap.put(4.011, -0.37); // 2.5 meters
        shooterMap.put(4.749, -0.39); // 3 meters
        shooterMap.put(5.381, -0.41); // 3.5 meters
        shooterMap.put(5.975, -0.43); // 4 meters
        shooterMap.put(6.473, -0.47); // 4.5 meters
        shooterMap.put(6.943, -0.49); // 5 meters
        shooterMap.put(7.139, -0.52); // 5.22 meters

        fieldTest = new Field2d();

        addRequirements(s_ShooterSubsystem);
    }

    private boolean isHighY(Pose2d botPose){
        if (botPose.getY() >= 4.021328){
            return true;
        }
        return false;
    }

    private Pose3d getShuttleTargetPose(){
        if (DriverStation.Alliance.Red.equals(DriverStation.getAlliance().get())) {
            if (isHighY(botPose)){
                return Constants.redNorthShuttleTarget;
            }
            else{
                return Constants.redSouthShuttleTarget;
            }
        }

        else{
            if (isHighY(botPose)){
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
            s_ShooterSubsystem.setFlywheels(-motorSpeed);
            s_ShooterSubsystem.setKicker(-0.2);

            if (s_ShooterSubsystem.isFlywheelReady(motorSpeed) && s_ShooterSubsystem.isHoodReadyToShoot() && s_ShooterSubsystem.isSwivelReadyToShoot()){ //another 0.5 sec delay
                s_ShooterSubsystem.setSpindexer(0.4, 0.1);
            }
            else {
                s_ShooterSubsystem.setSpindexer(0,0);
            }
        } else {
            s_ShooterSubsystem.setFlywheels(0);
            s_ShooterSubsystem.setKicker(0);
            s_ShooterSubsystem.setSpindexer(0, 0);
        }
    }

    private void autoShooting() {
        if (DriverStation.isAutonomousEnabled()) {
            if (DriverStation.Alliance.Blue.equals(DriverStation.getAlliance())) {
                if (s_Swerve.getEstimatedPosition().getX() <= Constants.blueAllianceLineX) {
                    s_ShooterSubsystem.setIsShooting(true);
                } else {
                    s_ShooterSubsystem.setIsShooting(false);
                }
            } else {
                if (s_Swerve.getEstimatedPosition().getX() >= Constants.redAllianceLineX) {
                    s_ShooterSubsystem.setIsShooting(true);
                } else {
                    s_ShooterSubsystem.setIsShooting(false);
                }
            }
        }
    }

    @Override
    public void execute() {

        botPose = s_Swerve.getEstimatedPosition();

        autoShooting();

        if (operator.getRawButtonPressed(XboxController.Button.kRightStick.value)) {
            isManual = !isManual;
        }

        if (isManual) {
            double manualSwivel = 1 * MathUtil.applyDeadband(operator.getRawAxis(XboxController.Axis.kRightX.value), Constants.stickDeadband);
            double manualHood = 1 * MathUtil.applyDeadband(operator.getRawAxis(XboxController.Axis.kRightY.value), Constants.stickDeadband);
            s_ShooterSubsystem.setSwivelSetpoint(s_ShooterSubsystem.getSwivelSetpoint() - manualSwivel);
            s_ShooterSubsystem.setHoodSetpoint(s_ShooterSubsystem.getHoodSetpoint() - manualHood);
            if (s_ShooterSubsystem.getIsShooting()) { 
                s_ShooterSubsystem.setFlywheels(SmartDashboard.getNumber("FlywheelSpeed", 0));
                s_ShooterSubsystem.setKicker(-.4);
            } else {
                s_ShooterSubsystem.setFlywheels(0);
                s_ShooterSubsystem.setKicker(0);
            }
            // attemptToShoot(motorSpeed);
            return;
        }
        
        boolean isInNeutral = s_Swerve.isInNeutral(botPose);

        if (isInNeutral){
            targetPose = getShuttleTargetPose();
        }
        else{
            targetPose = getHubTargetPose();
        }

        // targetPose = getHubTargetPose();

        TurretState initialState = calculateTurretWithPosition(targetPose);

        double firstPeriod = (2*initialState.initialVelocity*(Math.sin(initialState.hoodDegrees))) / 9.81;
        double botSpeedX = s_Swerve.getEstimatedFieldRelativeSpeeds().vxMetersPerSecond;
        double botSpeedY = s_Swerve.getEstimatedFieldRelativeSpeeds().vyMetersPerSecond;

        Pose3d secondPose = new Pose3d(targetPose.getX() + (botSpeedX*firstPeriod), targetPose.getY() + (botSpeedY*firstPeriod), targetPose.getZ(), new Rotation3d());
        TurretState secondState = calculateTurretWithPosition(secondPose);
        
        double secondPeriod = (2*secondState.initialVelocity*(Math.sin(secondState.hoodDegrees))) / 9.81;
        double periodError = firstPeriod - secondPeriod;
        // Pose3d secondPose = targetPose;
        // double periodError = firstPeriod;

        Pose3d thirdPose = new Pose3d(secondPose.getX() + (botSpeedX*(periodError)), secondPose.getY() + (botSpeedY*(periodError)), secondPose.getZ(), new Rotation3d());
        TurretState thirdState = calculateTurretWithPosition(thirdPose);



        // get the theta angle relative to robot rotation converted to encoder values
        double robotRelativeAngleDegrees = (thirdState.turretDegrees) % 360; //  - botPose.getRotation().getDegrees() + Constants.turretPoseRobotReletive.getRotation().getDegrees()

        if (robotRelativeAngleDegrees <= 0 && robotRelativeAngleDegrees >= -Constants.autoRotateRange) {
            isWithinAutoTurn = true;
        }
      
        if (robotRelativeAngleDegrees > 360) {
            robotRelativeAngleDegrees -= 360;
        }
      
        if (robotRelativeAngleDegrees < 0) {
            robotRelativeAngleDegrees += 360;
        }

        SmartDashboard.putNumber("Turret State Degrees", thirdState.turretDegrees);

        double distToTarget = Math.hypot(targetPose.toPose2d().relativeTo(thirdState.fieldRelativePose).getX(), targetPose.toPose2d().relativeTo(thirdState.fieldRelativePose).getY());

        if (distToTarget < 1.5) {
            robotRelativeAngleDegrees = 90;
        }

        SmartDashboard.putNumber("RRAngle Degrees", robotRelativeAngleDegrees);

        double robotRelativeSwivelEncoder = robotRelativeAngleDegrees * Constants.swivelEncoderPerDegrees;

        // TODO: Figure out velocity to motor speed scale irl (and if it's linear like this or not)
        // double motorSpeed = 2 * ((0.0100928 * (thirdState.initialVelocity * thirdState.initialVelocity)) + (0.00755809 * thirdState.initialVelocity));
        // double motorSpeed = shooterMap.get(thirdState.initialVelocity);
        double motorSpeed = ((-8.13 * Math.pow(10, -3)) * (Math.pow(thirdState.initialVelocity, 2))) + (0.044 * thirdState.initialVelocity) - 0.411;

        ledSubsystem.setIsRotationAligned(false);
        ledSubsystem.setIsRotationNearUnaligned(false);

       

        double fieldRelativeAngleDegrees = thirdState.turretDegrees % 360;
        double launchAngleDegrees = thirdState.hoodDegrees;

        SmartDashboard.putNumber("Launch Angle State", launchAngleDegrees);

        if (fieldRelativeAngleDegrees > 360) {
            fieldRelativeAngleDegrees -= 360;
        }
        if (fieldRelativeAngleDegrees < 0) {
            fieldRelativeAngleDegrees += 360;
        }

        // SmartDashboard.putNumber("ThetaDegrees", fieldRelativeAngleDegrees);

        // if where the swivel should be is within full range
        if (robotRelativeAngleDegrees >= 0 && robotRelativeAngleDegrees <= Constants.totalSwivelRangeDegrees) {
            ledSubsystem.setIsRotationAligned(true);
            s_ShooterSubsystem.setSwivelSetpoint(robotRelativeSwivelEncoder);

            // if where the swivel should be is within 20 degrees of being out of range (still in range)
            if (robotRelativeAngleDegrees <= 20 && robotRelativeAngleDegrees >= 0 || robotRelativeAngleDegrees <= Constants.totalSwivelRangeDegrees && robotRelativeAngleDegrees >= Constants.totalSwivelRangeDegrees - 20) {
                ledSubsystem.setIsRotationNearUnaligned(true);
            }
        } // if where the swivel should be is within 60 degrees out of range towards minimum
        else if (isWithinAutoTurn) {
            s_ShooterSubsystem.setSwivelSetpoint(0);
            isWithinAutoTurn = false;
        } // if where the swivel should be is within 60 degrees out of range towards maximum
        else if (robotRelativeAngleDegrees >= Constants.totalSwivelRangeDegrees && robotRelativeAngleDegrees <= Constants.totalSwivelRangeDegrees + Constants.autoRotateRange){
            s_ShooterSubsystem.setSwivelSetpoint(Constants.maximumSwivelEncoder);
        } // if where the swivel should be is out of range more than 60 degrees both ways
        else {
            s_ShooterSubsystem.setSwivelSetpoint(robotRelativeSwivelEncoder);
        }

        ledSubsystem.setIsHoodReady(s_ShooterSubsystem.isHoodReadyToShoot());
        ledSubsystem.setIsTurretAimed(s_ShooterSubsystem.isSwivelReadyToShoot());

        isUnderTrench = false;

        // if (((thirdState.fieldRelativePose.getX() >= Constants.redTrenchAreaLeftX && thirdState.fieldRelativePose.getX() <= Constants.redTrenchAreaRightX) || (thirdState.fieldRelativePose.getX() >= Constants.blueTrenchAreaLeftX && thirdState.fieldRelativePose.getX() <= Constants.blueTrenchAreaRightX)) && (thirdState.fieldRelativePose.getY() >= Constants.TrenchAreaTopY || thirdState.fieldRelativePose.getY() <= Constants.TrenchAreaBottomY)) {
        //     s_ShooterSubsystem.setHoodSetpoint(launchAngleDegrees * Constants.hoodEncoderPerDegree);
        //     isUnderTrench = true;
        // } 
        // else 
        if (isInNeutral){
            s_ShooterSubsystem.setHoodSetpoint(Constants.maximumHoodEncoder);
        }
        else {
            s_ShooterSubsystem.setHoodSetpoint((launchAngleDegrees - Constants.minimumHoodAngle) * Constants.hoodEncoderPerDegree);
        }

        // SmartDashboard.putNumber("Flywheel Speed", motorSpeed);
        // SmartDashboard.putNumber("Initial Velocity", thirdState.initialVelocity);

        //attemptToShoot(motorSpeed);

    }

    private TurretState calculateTurretWithPosition(Pose3d targetPosition){

        botPose = s_Swerve.getEstimatedPosition(); // TODO: Uncomment this line once Swerve is merged into main!!!
        // SmartDashboard.putNumber("EstimatorRot", s_Swerve.getEstimatedPosition().getRotation().getDegrees());
        SmartDashboard.putString("Calc BotPose", botPose.toString());
        double botRotDegrees = botPose.getRotation().getDegrees() % 360;

        if (botRotDegrees > 360) {
            botRotDegrees -= 360;
        }
        if (botRotDegrees < 0) {
            botRotDegrees += 360;
        }

        //fieldTest.setRobotPose(botPose);
        // SmartDashboard.putData("TestField", fieldTest);


        // double turretX = SmartDashboard.getNumber("TurretX", 0);
        // double turretY = SmartDashboard.getNumber("TurretY", 0);
        // double turretDist = Math.sqrt((turretY * turretY) + (turretX * turretX));
        // double turretDist = Math.sqrt((Constants.turretPoseRobotReletive.getY() * Constants.turretPoseRobotReletive.getY()) + (Constants.turretPoseRobotReletive.getX() * Constants.turretPoseRobotReletive.getX()));

        // double turretTheta = Math.atan2(Constants.turretPoseRobotReletive.getY(), Constants.turretPoseRobotReletive.getX()) + botPose.getRotation().getRadians() - (Math.PI/2);
        // double turretTheta = Rotation2d.fromRadians(Math.atan2(turretY,turretX) - (Math.PI/2)).getRadians();
        // SmartDashboard.putNumber("TurretTheta", turretTheta * (180/Math.PI));

        double turretOffsetX = Constants.turretPoseRobotReletive.getX();
        double turretOffsetY = Constants.turretPoseRobotReletive.getY();
        double botX = (turretOffsetX * botPose.getRotation().getCos()) - (turretOffsetY * botPose.getRotation().getSin());
        double botY = (turretOffsetY * botPose.getRotation().getCos()) + (turretOffsetX * botPose.getRotation().getSin());

        Pose2d turretPoseFieldRelative = new Pose2d(botX + botPose.getX(), botY + botPose.getY(), Rotation2d.fromDegrees(botRotDegrees + Constants.turretPoseRobotReletive.getRotation().getDegrees()));
        SmartDashboard.putString("Turret Pose", turretPoseFieldRelative.toString());

        // Distance in x and y axis respectively
        double dx = targetPosition.getX() - turretPoseFieldRelative.getX();
        double dy = targetPosition.getY() - turretPoseFieldRelative.getY();

        SmartDashboard.putNumber("dX", dx);
        SmartDashboard.putNumber("dY", dy);
        // angle in radians of the theoretical setpoint while stood still.
        double thetaDegrees = Math.toDegrees(Math.atan2(dy, dx));
        SmartDashboard.putNumber("Theta Degrees", thetaDegrees);
        // Rotation2d thetaDegrees = getTurretTargetAngle(botPose, targetPosition);

        // System.out.println("Bot Pose \n" + botPose);
        // System.out.println("Target Pose \n" + targetPose);

        // distance away from center point of the turret to the center of the hub
        double distanceFromTarget = Math.hypot(dx, dy);
        SmartDashboard.putNumber("Distance From Hub", distanceFromTarget);

        // hypothetically, this math should give the launcher angle in degrees from 75 to 85 scaled to distance away from the center of the hub
        double launchAngleDegrees = MathUtil.clamp(((distanceFromTarget / Constants.maximumHubDist) * (Constants.maximumHoodShotDegrees - Constants.minimumHoodShotDegrees)) + Constants.minimumHoodShotDegrees, Constants.minimumHoodShotDegrees, Constants.maximumHoodShotDegrees);
        //65.0; //((hypotenuse - Constants.minimumHubDist) / (Constants.maximumHubDist - Constants.minimumHubDist)) * (Constants.maximumHoodAngle - Constants.minimumHoodAngle) + Constants.minimumHoodAngle;
        // SmartDashboard.putNumber("Launch Angle", launchAngleDegrees);

        // Distance the ball needs to hit for the ball to hit the height and position of the hub along it's parabola
        double shootingTargetDistance = distanceFromTarget + (targetPosition.getZ() / Math.tan(launchAngleDegrees));

        // Initial velocity in m/s that the ball should have to travel to score (9.81 is gravity)
        double vO = Math.sqrt((shootingTargetDistance * 9.81) / Math.sin(2 * Math.toRadians(launchAngleDegrees)));

        return new TurretState(thetaDegrees, launchAngleDegrees, vO, turretPoseFieldRelative);

    }

    public Rotation2d getTurretTargetAngle(Pose2d robotPose, Pose3d hubPose) {

        // Convert hub to 2D
        Pose2d hub2d = hubPose.toPose2d();

        // Get turret pose relative to robot
        Pose2d turretRelative = Constants.turretPoseRobotReletive; //new Pose2d(SmartDashboard.getNumber("TurretX", 0.1), SmartDashboard.getNumber("TurretY", 0.1), Rotation2d.fromDegrees(90));

        // Convert turret pose into field coordinates
        Pose2d turretFieldPose = robotPose.transformBy(
            new Transform2d(
                turretRelative.getTranslation(),
                turretRelative.getRotation()
            )
        );

        // Transform from turret -> hub
        Transform2d turretToHub = new Transform2d(
            turretFieldPose,
            hub2d
        );

        // Angle turret must rotate
        // System.out.println("Turret Calculation Pose \n" + turretToHub);
        return turretToHub.getTranslation().getAngle().plus(Rotation2d.fromDegrees(180));
    }

}
