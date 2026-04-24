package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeExtensionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class IntakeExtensionDefault extends Command {

    private IntakeExtensionSubsystem intakeExtensionSubsystem;

    private Joystick driver;

    private int timer = 0;

    public IntakeExtensionDefault(Joystick driver) {

        this.driver = driver;
        intakeExtensionSubsystem = IntakeExtensionSubsystem.getInstance();

        addRequirements(intakeExtensionSubsystem);
    }

    @Override
    public void execute() {
        double setpoint = intakeExtensionSubsystem.getIntakeSetpoint();
        if (driver.getPOV() == 0) {
            setpoint += 0.1;
        } else if (driver.getPOV() == 180) {
            setpoint -= 0.1;
        }

        if (ShooterSubsystem.getInstance().getIsShooting()) {
            if (timer <= 30 && intakeExtensionSubsystem.getIsJiggling()) {
                setpoint = -2;
            } else {
                setpoint = Constants.minimumIntakeEncoder;
            }
            timer += 1;
            if (timer >= 60) {
                timer = 0;
            }

            System.out.println(timer);
        }
        // Always call ts to update position/always calculate
        intakeExtensionSubsystem.setExtensionSetpoint(setpoint);
    }
}
