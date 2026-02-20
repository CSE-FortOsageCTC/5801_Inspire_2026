package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.sql.Driver;

import org.w3c.dom.css.RGBColor;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import frc.robot.subsystems.Swerve;

public class LEDSubsystem extends SubsystemBase {
    private CANdle candle1 = new CANdle(42);
    private RainbowAnimation rainbowAnimation = new RainbowAnimation(1, .8, 31);
    // private TwinkleAnimation larsonAnimation = new TwinkleAnimation(65,105,225);
    private StrobeAnimation strobeAnimation = new StrobeAnimation(65, 105, 225, 255, 0.2,31);
    private RgbFadeAnimation rgbFadeAnimation = new RgbFadeAnimation(255, 0.8, 31);

    private static LEDSubsystem ledSubsystem;
    private IntakeSubsystem intakeSubsystem;

    private Debouncer isCoralDebouncerLeft;
    private Debouncer isCoralDebouncerRight;

    private double timer;
    private boolean isStrobing;

    public boolean isRed;
    public boolean isBlue;
    public boolean isYellow;
    public boolean isGreen;


    private int[] fullBlue= {0, 0, 255};

    private int[] fullRed = {255, 0, 0};

    public static LEDSubsystem getInstance() {
        if (ledSubsystem == null) {
            ledSubsystem = new LEDSubsystem();
        }
        return ledSubsystem;
    }

    private LEDSubsystem() {
        isRed = DriverStation.getAlliance().get().equals(Alliance.Red);
        intakeSubsystem = IntakeSubsystem.getInstance();
        candle1.setLEDs(0, 0, 0);
        candle1.configBrightnessScalar(1);
        candle1.clearAnimation(1);
        candle1.clearAnimation(2);

        isCoralDebouncerLeft = new Debouncer(0.05);
        isCoralDebouncerRight = new Debouncer(0.05);

        timer = 0;
        isStrobing = false;
    }

    public void setStrobe() {
        isStrobing = true;
        timer++;
        candle1.animate(strobeAnimation, 1);
        if (timer > 40) {
            isStrobing = false;
            timer = 0;
        }
    }

    public void setRainbow() {
        candle1.animate(rainbowAnimation, 1);
    }

    public void setRgbFade() {
        candle1.animate(rgbFadeAnimation, 1);
    }

    public void setBlack() {
        candle1.clearAnimation(1);
        candle1.setLEDs(0, 0, 0);
        timer = 0;
    }

    public void setColor(int r, int g, int b) {
        candle1.clearAnimation(1);
        candle1.setLEDs(r, g, b);
        timer = 0;
    }

    @Override
    public void periodic() {
        // setColor(rgbColor[0], rgbColor[1], rgbColor[2]);
        // candle1.setLEDs(255, 255, 0);
        //logic for LEDs: turn red if swerve is unaligned (180 degrees), yellow if close, blue if aligned and close but not ready to shoot, green ready to shoot (hood and align) - logic as i understand it
        if (isRed) {
            setColor(255, 0, 0);
        }
        if (isBlue) {
            setColor(0, 0, 255);
        }
        if (isYellow) {
            setColor (255, 255, 0);
        }
        if (isGreen) {
            setColor (0, 255, 0);
        }
    }    
}