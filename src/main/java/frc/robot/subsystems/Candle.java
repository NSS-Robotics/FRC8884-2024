package frc.robot.subsystems;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Candle extends SubsystemBase {

    CANdle candleLeft = new CANdle(Constants.CandleConstants.candleLeft);
    CANdle candleRight = new CANdle(Constants.CandleConstants.candleRight);

    RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.6, 64);
    TwinkleAnimation twinkleAnim = new TwinkleAnimation(0, 255, 0);
    StrobeAnimation strobeAnim = new StrobeAnimation(0, 255, 0);

    ColorFlowAnimation colorFlowAnim = new ColorFlowAnimation(
        0,
        0,
        0,
        0,
        0.3,
        111,
        Direction.Forward
    );

    CANdleConfiguration config = new CANdleConfiguration();

    public Candle() {
        config.brightnessScalar = 0.5;
        candleLeft.configAllSettings(config);
        candleRight.configAllSettings(config);
    }

    public void setLEDs(int R, int G, int B) {
        candleLeft.setLEDs(R, G, B);
        candleRight.setLEDs(R, G, B);
    }

    public void rainbow() {
        candleLeft.animate(rainbowAnim);
        candleRight.animate(rainbowAnim);
    }

    public void twinkle() {
        candleLeft.animate(twinkleAnim);
        candleRight.animate(twinkleAnim);
    }

    public void strobe(int r, int g, int b) {
        strobeAnim.setR(r);
        strobeAnim.setB(b);
        strobeAnim.setG(g);
        candleLeft.animate(strobeAnim);
        candleRight.animate(strobeAnim);
    }

    public void flow(int r, int g, int b) {
        colorFlowAnim.setR(r);
        colorFlowAnim.setG(g);
        colorFlowAnim.setB(b);
        candleLeft.animate(colorFlowAnim);
        candleRight.animate(colorFlowAnim);
    }

    public void ledsOff(){
        candleLeft.clearAnimation(0);
        candleRight.clearAnimation(0);
        config.brightnessScalar = 0;
        candleLeft.configAllSettings(config);
        candleRight.configAllSettings(config);
    }
}
