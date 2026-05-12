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
import frc.robot.VelocityEstimator;
import frc.robot.subsystems.IntakeExtensionSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve;

public class ShooterDefault extends Command {
    
    private ShooterSubsystem s_ShooterSubsystem;
    private LEDSubsystem ledSubsystem;
    private IntakeExtensionSubsystem s_IntakeExtensionSubsystem;

    private Swerve s_Swerve;

    private Pose2d botPose;

    private Field2d fieldTest;

    private InterpolatingDoubleTreeMap shooterMap;

    private Joystick operator;
    private Pose3d targetPose;

    private boolean isUnderTrench = false;

    private boolean isWithinAutoTurn = false;

    private boolean isSwivelInBounds = false;

    private boolean isManual = false;

    public ShooterDefault(Joystick operator) {
        s_ShooterSubsystem = ShooterSubsystem.getInstance();
        s_IntakeExtensionSubsystem = IntakeExtensionSubsystem.getInstance();
        this.operator = operator;
        s_Swerve = Swerve.getInstance();
        ledSubsystem = LEDSubsystem.getInstance();

        // SmartDashboard.putNumber("TestX", 0);
        // SmartDashboard.putNumber("TestY", 0);
        // SmartDashboard.putNumber("TestDegrees", 0);
        SmartDashboard.putNumber("TurretX", 0);
        SmartDashboard.putNumber("TurretY", 0);
        SmartDashboard.putNumber("FlywheelSpeed", 0);
        SmartDashboard.putNumber("FlyWheel Mult", 25);

        shooterMap = new InterpolatingDoubleTreeMap();

        shooterMap.put(0.0, 0.0); // 0 meters
        shooterMap.put(5.32, -0.35); // 1 meter
        shooterMap.put(5.77, -0.35); // 1.5 meters
        shooterMap.put(6.22, -0.36); // 2 meters
        shooterMap.put(6.49, -0.36); // 2.5 meters
        shooterMap.put(6.8, -0.39); // 3 meters
        shooterMap.put(7.1, -0.395); // 3.5 meters
        shooterMap.put(7.403, -0.4075); // 4 meters
        shooterMap.put(7.75, -0.42); // 4.5 meters
        shooterMap.put(8.07, -0.45); // 5 meters
        shooterMap.put(10.0, -0.6); // Max

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
            s_ShooterSubsystem.setKicker(0.2);

            if (s_ShooterSubsystem.isFlywheelReady(motorSpeed) && s_ShooterSubsystem.isHoodReadyToShoot() && s_ShooterSubsystem.isSwivelReadyToShoot() && isSwivelInBounds){ //another 0.5 sec delay
                s_ShooterSubsystem.setSpindexer(0.75);
            }
            else {
                s_ShooterSubsystem.setSpindexer(0);
            }
        } else {
            s_ShooterSubsystem.setFlywheels(0);
            s_ShooterSubsystem.setKicker(0);
            s_ShooterSubsystem.setSpindexer(0);
        }
    }

    private void autoShooting(boolean inNeutral) {
        if (DriverStation.isAutonomousEnabled() && !inNeutral) {
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
        } else if (DriverStation.isAutonomousEnabled()) {
            s_ShooterSubsystem.setIsShooting(false);
        }
    }

    @Override
    public void execute() {

        botPose = s_Swerve.getEstimatedPosition();
        // botPose = new Pose2d(SmartDashboard.getNumber("TurretX", 0), SmartDashboard.getNumber("TurretY", 0), new Rotation2d());

        if (s_IntakeExtensionSubsystem.getExtensionState()) {

            if (operator.getRawButtonPressed(XboxController.Button.kRightStick.value)) {
                isManual = !isManual;
            }

            if (isManual) {
                double manualSwivel = 1 * MathUtil.applyDeadband(operator.getRawAxis(XboxController.Axis.kRightX.value), Constants.stickDeadband);
                double manualHood = 1 * MathUtil.applyDeadband(operator.getRawAxis(XboxController.Axis.kRightY.value), Constants.stickDeadband);
                s_ShooterSubsystem.setSwivelSetpoint(s_ShooterSubsystem.getSwivelSetpoint() - manualSwivel);
                s_ShooterSubsystem.setHoodSetpoint(s_ShooterSubsystem.getHoodSetpoint() - manualHood);
                if (s_ShooterSubsystem.getIsShooting()) { 
                    s_ShooterSubsystem.setFlywheels(
                        SmartDashboard.getNumber("FlywheelSpeed", 0));
                    s_ShooterSubsystem.setKicker(0.4);
                    System.out.println("Shoot!");
                } else {
                    s_ShooterSubsystem.setFlywheels(0);
                    s_ShooterSubsystem.setKicker(0);
                    System.out.println("No Shoot!");
                }
                //attemptToShoot(1.0);
                return;
            }
            
            boolean isInNeutral = s_Swerve.isInNeutral(botPose);

            if (isInNeutral){
                targetPose = getShuttleTargetPose();
            }
            else{
                targetPose = getHubTargetPose();
            }

            // autoShooting(isInNeutral);

            // targetPose = getHubTargetPose();

            TurretState initialState = calculateTurretWithPosition(targetPose);

            // double firstPeriod = (2*initialState.initialVelocity*(Math.sin(initialState.hoodDegrees))) / 9.81;
            VelocityEstimator velocityEstimator = s_Swerve.getVelocityEstimator();
            double botSpeedX = MathUtil.clamp(velocityEstimator.getVelocity().vxMetersPerSecond, -Constants.Swerve.maxActualSpeed, Constants.Swerve.maxActualSpeed);
            double botSpeedY = MathUtil.clamp(velocityEstimator.getVelocity().vyMetersPerSecond, -Constants.Swerve.maxActualSpeed, Constants.Swerve.maxActualSpeed);
            // double botSpeedX = s_Swerve.getEstimatedFieldRelativeSpeeds().vxMetersPerSecond;
            // double botSpeedY = s_Swerve.getEstimatedFieldRelativeSpeeds().vyMetersPerSecond;

            // Pose3d secondPose = new Pose3d(targetPose.getX() + (botSpeedX*firstPeriod), targetPose.getY() + (botSpeedY*firstPeriod), targetPose.getZ(), new Rotation3d());
            // TurretState secondState = calculateTurretWithPosition(secondPose);
            
            // double secondPeriod = (2*secondState.initialVelocity*(Math.sin(secondState.hoodDegrees))) / 9.81;
            // double periodError = firstPeriod - secondPeriod;
            // Pose3d secondPose = targetPose;
            // double periodError = firstPeriod;

            //Pose3d thirdPose = new Pose3d(secondPose.getX() + (botSpeedX*(periodError)), secondPose.getY() + (botSpeedY*(periodError)), secondPose.getZ(), new Rotation3d());
            TurretState thirdState = initialState;//calculateTurretWithPosition(thirdPose);


            double sanitizedBotRotation = ((botPose.getRotation().getDegrees() % 360) + 360) % 360;
            // get the theta angle relative to robot rotation converted to encoder values
            double robotRelativeAngleDegrees = ((thirdState.turretDegrees) - sanitizedBotRotation + Constants.turretPoseRobotReletive.getRotation().plus(s_Swerve.getTurretHeading().times(2)).getDegrees()) % 360;
            SmartDashboard.putNumber("BotRot", sanitizedBotRotation);
            isWithinAutoTurn = false; //robotRelativeAngleDegrees <= (Constants.minimumSwivelEncoder / Constants.swivelEncoderPerDegrees) && robotRelativeAngleDegrees >= -Constants.autoRotateRange;
        
            // if (robotRelativeAngleDegrees > 180) {
            //     robotRelativeAngleDegrees -= 180;
            // }
        
            if (robotRelativeAngleDegrees < 0) {
                robotRelativeAngleDegrees += 360;
            }

            SmartDashboard.putNumber("Turret State Degrees", thirdState.turretDegrees);

            double distToTarget = Math.hypot(targetPose.toPose2d().relativeTo(thirdState.fieldRelativePose).getX(), targetPose.toPose2d().relativeTo(thirdState.fieldRelativePose).getY());

            if (distToTarget < 0.75) {
                robotRelativeAngleDegrees = 90;
            }

            SmartDashboard.putNumber("RRAngle Degrees", robotRelativeAngleDegrees);

            if (robotRelativeAngleDegrees > (Constants.maximumSwivelEncoder / Constants.swivelEncoderPerDegrees)) {
                robotRelativeAngleDegrees -= 360;
            }

            double robotRelativeSwivelEncoder = robotRelativeAngleDegrees * Constants.swivelEncoderPerDegrees;

            // TODO: Figure out velocity to motor speed scale irl (and if it's linear like this or not)
            // double motorSpeed = ((-0.0116 * (thirdState.initialVelocity * thirdState.initialVelocity)) + (0.119 * thirdState.initialVelocity) - 0.654);
            double speedMult = 1.02; //(SmartDashboard.getNumber("FlyWheel Mult", 25) * 0.01);
            double motorSpeed = shooterMap.get(thirdState.initialVelocity) * speedMult;
            // double motorSpeed = (-0.0432 * (Math.pow(thirdState.initialVelocity, 2))) + (0.468 * thirdState.initialVelocity) - 1.7;

            SmartDashboard.putNumber("FlyWheel Calc Speed", motorSpeed);

            ledSubsystem.setIsRotationAligned(false);
            ledSubsystem.setIsRotationNearUnaligned(false);

        

            // double fieldRelativeAngleDegrees = thirdState.turretDegrees % 360;
            double launchAngleDegrees = thirdState.hoodDegrees;

            SmartDashboard.putNumber("Launch Angle State", launchAngleDegrees);

            // if (fieldRelativeAngleDegrees > 360) {
            //     fieldRelativeAngleDegrees -= 360;
            // }
            // if (fieldRelativeAngleDegrees < 0) {
            //     fieldRelativeAngleDegrees += 360;
            // }

            // SmartDashboard.putNumber("ThetaDegrees", fieldRelativeAngleDegrees);

            // if where the swivel should be is within full range
            if (robotRelativeAngleDegrees >= Constants.minimumSwivelAngle && robotRelativeAngleDegrees <= Constants.maximumSwivelAngle) {
                ledSubsystem.setIsRotationAligned(true);
                s_ShooterSubsystem.setSwivelSetpoint(MathUtil.clamp(robotRelativeSwivelEncoder, Constants.minimumSwivelEncoder, Constants.maximumSwivelEncoder));

                // if where the swivel should be is within 20 degrees of being out of range (still in range)
                if (robotRelativeAngleDegrees <= 20 && robotRelativeAngleDegrees >= 0 || robotRelativeAngleDegrees <= Constants.totalSwivelRangeDegrees && robotRelativeAngleDegrees >= Constants.totalSwivelRangeDegrees - 20) {
                    ledSubsystem.setIsRotationNearUnaligned(true);
                }
                isSwivelInBounds = true;
            } // if where the swivel should be is within 60 degrees out of range towards minimum
            else if (isWithinAutoTurn) {
                s_ShooterSubsystem.setSwivelSetpoint(0);
            } // if where the swivel should be is within 60 degrees out of range towards maximum
            // else if (robotRelativeAngleDegrees >= Constants.totalSwivelRangeDegrees && robotRelativeAngleDegrees <= Constants.totalSwivelRangeDegrees + Constants.autoRotateRange){
            //     s_ShooterSubsystem.setSwivelSetpoint(Constants.maximumSwivelEncoder);
            // } // if where the swivel should be is out of range more than 60 degrees both ways
            else {
                isSwivelInBounds = false;
                s_ShooterSubsystem.setSwivelSetpoint(MathUtil.clamp(robotRelativeSwivelEncoder, Constants.minimumSwivelEncoder, Constants.maximumSwivelEncoder));
            }

            // s_ShooterSubsystem.setSwivelSetpoint(MathUtil.clamp(20, Constants.minimumSwivelEncoder, Constants.maximumSwivelEncoder));

            ledSubsystem.setIsHoodReady(s_ShooterSubsystem.isHoodReadyToShoot());
            ledSubsystem.setIsTurretAimed(s_ShooterSubsystem.isSwivelReadyToShoot());

            isUnderTrench = false;

            // if (((thirdState.fieldRelativePose.getX() >= Constants.redTrenchAreaLeftX && thirdState.fieldRelativePose.getX() <= Constants.redTrenchAreaRightX) || (thirdState.fieldRelativePose.getX() >= Constants.blueTrenchAreaLeftX && thirdState.fieldRelativePose.getX() <= Constants.blueTrenchAreaRightX)) && (thirdState.fieldRelativePose.getY() >= Constants.TrenchAreaTopY || thirdState.fieldRelativePose.getY() <= Constants.TrenchAreaBottomY)) {
            //     s_ShooterSubsystem.setHoodSetpoint(launchAngleDegrees * Constants.hoodEncoderPerDegree);
            //     isUnderTrench = true;
            // } 
            // else 
            // if (isInNeutral){
            //     s_ShooterSubsystem.setHoodSetpoint(Constants.maximumHoodEncoder);
            // }
            // else {
            //     s_ShooterSubsystem.setHoodSetpoint((Constants.totalHoodRangeDegrees - (launchAngleDegrees - Constants.minimumHoodAngle)) * Constants.hoodEncoderPerDegree);
            // }

            s_ShooterSubsystem.setHoodSetpoint((Constants.totalHoodRangeDegrees - (launchAngleDegrees - Constants.minimumHoodAngle)) * Constants.hoodEncoderPerDegree);
            

            // SmartDashboard.putNumber("Flywheel Speed", motorSpeed);
            SmartDashboard.putNumber("Initial Velocity", thirdState.initialVelocity);

            // attemptToShoot(SmartDashboard.getNumber("FlywheelSpeed", 0));
            attemptToShoot(motorSpeed);
        } else {
            s_ShooterSubsystem.setHoodSetpoint(s_ShooterSubsystem.getHoodEncoder());
            s_ShooterSubsystem.setSwivelSetpoint(0);
        }

    }

    private TurretState calculateTurretWithPosition(Pose3d targetPosition){

        botPose = s_Swerve.getEstimatedPosition(); // TODO: Uncomment this line once Swerve is merged into main!!!
        
        // botPose = new Pose2d(SmartDashboard.getNumber("TurretX", 0), SmartDashboard.getNumber("TurretY", 0), new Rotation2d());

        // SmartDashboard.putNumber("EstimatorRot", s_Swerve.getEstimatedPosition().getRotation().getDegrees());
        // SmartDashboard.putString("Calc BotPose", botPose.toString());

        Rotation2d botPoseRotation = botPose.getRotation().rotateBy(s_Swerve.getTurretHeading().times(-2));

        double botRotDegrees = botPoseRotation.getDegrees() % 180;

        botPoseRotation = Rotation2d.fromDegrees(((botPoseRotation.getDegrees() % 360) + 360) % 360);
        SmartDashboard.putNumber("Bot Pose Rotation", botPoseRotation.getDegrees());
        // if (botRotDegrees > 360) {
        //     botRotDegrees -= 360;
        // }
        // if (botRotDegrees < 0) {
        //     botRotDegrees += 360;
        // }

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
        double botX = (turretOffsetX * botPoseRotation.getCos()) - (turretOffsetY * botPoseRotation.getSin());
        double botY = (turretOffsetY * botPoseRotation.getCos()) + (turretOffsetX * botPoseRotation.getSin());

        Pose2d turretPoseFieldRelative = new Pose2d(botX + botPose.getX(), botY + botPose.getY(), Rotation2d.fromDegrees(botRotDegrees + Constants.turretPoseRobotReletive.getRotation().getDegrees()));
        // SmartDashboard.putString("Turret Pose", turretPoseFieldRelative.toString());

        // Distance in x and y axis respectively
        double dx = targetPosition.getX() - turretPoseFieldRelative.getX();
        double dy = targetPosition.getY() - turretPoseFieldRelative.getY();

        SmartDashboard.putNumber("dX", dx);
        SmartDashboard.putNumber("dY", dy);
        // angle in radians of the theoretical setpoint while stood still.
        double thetaDegrees = Math.toDegrees(Math.atan2(dy, dx));
        // SmartDashboard.putNumber("Theta Degrees", thetaDegrees);
        //double thetaDegrees = getTurretTargetAngle(turretPoseFieldRelative, targetPosition).getDegrees();

        // System.out.println("Bot Pose \n" + botPose);
        // System.out.println("Target Pose \n" + targetPose);

        // distance away from center point of the turret to the center of the hub
        double distanceFromTarget = Math.hypot(dx, dy);
        SmartDashboard.putNumber("Distance From Hub", distanceFromTarget);

        // hypothetically, this math should give the launcher angle in degrees from 75 to 85 scaled to distance away from the center of the hub
        double launchAngleDegrees = MathUtil.clamp(((1 - (distanceFromTarget / Constants.maximumHubDist)) * (Constants.maximumHoodShotDegrees - Constants.minimumHoodShotDegrees)) + Constants.minimumHoodShotDegrees, Constants.minimumHoodShotDegrees, Constants.maximumHoodShotDegrees);
        //65.0; //((hypotenuse - Constants.minimumHubDist) / (Constants.maximumHubDist - Constants.minimumHubDist)) * (Constants.maximumHoodAngle - Constants.minimumHoodAngle) + Constants.minimumHoodAngle;
        // SmartDashboard.putNumber("Launch Angle", launchAngleDegrees);

        // Distance the ball needs to hit for the ball to hit the height and position of the hub along it's parabola
        double shootingTargetDistance = distanceFromTarget + (targetPosition.getZ() / Math.tan(Math.toRadians(launchAngleDegrees)));

        // Initial velocity in m/s that the ball should have to travel to score (9.81 is gravity)
        double vO = Math.sqrt((shootingTargetDistance * 9.81) / Math.sin(2 * Math.toRadians(launchAngleDegrees)));

        return new TurretState(thetaDegrees, launchAngleDegrees, vO, turretPoseFieldRelative);

    }

    public Rotation2d getTurretTargetAngle(Pose2d turretPose, Pose3d hubPose) {

        // Convert hub to 2D
        Pose2d hub2d = hubPose.toPose2d();
        

        // Transform from turret -> hub
        Transform2d turretToHubTransform = new Transform2d(
            turretPose,
            hub2d
        );
        Pose2d turretToHubPose = turretPose.relativeTo(hub2d);

        // Angle turret must rotate
        // System.out.println("Turret Calculation Pose \n" + turretToHub);
        // return turretToHubTransform.getRotation();
        return turretToHubPose.getRotation();
    }

}
