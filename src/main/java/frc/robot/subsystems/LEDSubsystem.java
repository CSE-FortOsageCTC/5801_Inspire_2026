package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;

import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
    private CANdle candle1 = new CANdle(42); //Change this when we know
    private RainbowAnimation rainbowAnimation = new RainbowAnimation(0,  Constants.numberOfLEDs);
    // private TwinkleAnimation larsonAnimation = new TwinkleAnimation(65,105,225);
    private StrobeAnimation strobeAnimation = new StrobeAnimation(0, Constants.numberOfLEDs);
    private RgbFadeAnimation rgbFadeAnimation = new RgbFadeAnimation(0, Constants.numberOfLEDs);

    private static LEDSubsystem ledSubsystem;

    private double timer;
    private boolean isStrobing;

    private boolean isClimbAligned = false;
    private boolean isTurretAimed = false;
    private boolean isRotationNearUnaligned = false;
    private boolean isRotationAligned = false;
    private boolean isHoodReady = false;
    
    public static LEDSubsystem getInstance() {
        if (ledSubsystem == null) {
            ledSubsystem = new LEDSubsystem();
        }
        return ledSubsystem;
    }

    private LEDSubsystem() {
        setColor(0, 0, 0);
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

    public void setRed() {
        setColor(255, 0, 0);
    }

    public void setBlue() {
        setColor(0, 0, 255);
    }

    public void setGreen() {
        setColor(0, 255, 0);
    }

    public void setYellow() {
        setColor(255, 255, 0);
    }

    public void setIsClimbAligned(boolean isClimbAligned) {
        this.isClimbAligned = isClimbAligned;
    }

    public void setIsTurretAimed(boolean isTurretAimed) {
        this.isTurretAimed = isTurretAimed;
    }
    
    public void setIsRotationNearUnaligned(boolean isRotationNearUnaligned) {
        this.isRotationNearUnaligned = isRotationNearUnaligned;
    }

    public void setIsRotationAligned(boolean isRotationAligned) {
        this.isRotationAligned = isRotationAligned;
    }

    public void setIsHoodReady(boolean isHoodReady) {
        this.isHoodReady = isHoodReady;
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
        else if(isRotationNearUnaligned && isTurretAimed && isHoodReady)
        {
            setYellow();
        }
        else if(isRotationAligned && isTurretAimed && isHoodReady)
        {
            setGreen();
        }
        else if(!isTurretAimed && isRotationAligned)
        {
            setBlue();
        }
        else
        {
            setRed();
        }
    }
}