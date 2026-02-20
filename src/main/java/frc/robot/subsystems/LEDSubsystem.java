package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.sql.Driver;

import org.w3c.dom.css.RGBColor;

import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdle;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class LEDSubsystem extends SubsystemBase {
    private CANdle candle1 = new CANdle(42);
    private RainbowAnimation rainbowAnimation = new RainbowAnimation(1,  31);
    // private TwinkleAnimation larsonAnimation = new TwinkleAnimation(65,105,225);
    private StrobeAnimation strobeAnimation = new StrobeAnimation(65, 105);
    private RgbFadeAnimation rgbFadeAnimation = new RgbFadeAnimation(255, 31);

    private static LEDSubsystem ledSubsystem;
    private IntakeSubsystem intakeSubsystem;

    private Debouncer isCoralDebouncerLeft;
    private Debouncer isCoralDebouncerRight;

    private double timer;
    private boolean isStrobing;

    private boolean isClimbAligned = false;
    private boolean isTurretAimed = false;
    private boolean isRotationNearUnaligned = false;
    private boolean isRotationAligned = false;

    private SolidColor red = new SolidColor(0, Constants.numberOfLEDs);

    private int[] fullBlue= {0, 0, 255};

    private int[] fullRed = {255, 0, 0};

    public static LEDSubsystem getInstance() {
        if (ledSubsystem == null) {
            ledSubsystem = new LEDSubsystem();
        }
        return ledSubsystem;
    }

    private LEDSubsystem() {
        intakeSubsystem = IntakeSubsystem.getInstance();
        red.withColor(new RGBWColor(255, 0, 0));
        candle1.setControl(red);
        clearAnimation();
        
        timer = 0;
        isStrobing = false;
    }

    private void clearAnimation() {
        candle1.setControl(new EmptyAnimation(1));
        candle1.setControl(new EmptyAnimation(2));
    }

    public void setStrobe() {
        isStrobing = true;
        timer++;
        candle1.setControl(strobeAnimation);
        if (timer > 40) {
            isStrobing = false;
            timer = 0;
        }
    }

    public void setRainbow() {
        candle1.setControl(rainbowAnimation);
    }

    public void setRgbFade() {
        candle1.setControl(rgbFadeAnimation);
    }

    public void setBlack() {
        setColor(0, 0, 0);
        timer = 0;
    }

    public void setColor(int r, int g, int b) {
        clearAnimation();
        SolidColor color = new SolidColor(0, Constants.numberOfLEDs);
        color.withColor(new RGBWColor(r, g, b));
        candle1.setControl(color);
        timer = 0;
    }

    @Override
    public void periodic() {
        // setColor(rgbColor[0], rgbColor[1], rgbColor[2]);
        // candle1.setLEDs(255, 255, 0);
        //logic for LEDs: turn red if swerve is unaligned (180 degrees), yellow if close, blue if aligned and close but not ready to shoot, green ready to shoot (hood and align) - logic as i understand it
        if(isClimbAligned)
        {
            setRainbow();
        }
        else if(isRotationAligned && isTurretAimed)
        {
            setColor(0, 255, 0);
        }
        else if(isRotationNearUnaligned && isTurretAimed)
        {
            setColor(0, 0, 255);
        }
        else if(isRotationAligned)
        {
            setColor(255, 255, 0);
        }
        else
        {
            setColor(255, 0, 0);
        }
    }
}