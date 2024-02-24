package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Candle extends SubsystemBase{
    
    CANdle candleLeft = new CANdle(Constants.CandleConstants.candleLeft);
    CANdle candleRight = new CANdle(Constants.CandleConstants.candleRight);

    RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.5, 64);
    TwinkleAnimation twinkleAnim = new TwinkleAnimation(0, 255, 0);
    CANdleConfiguration config = new CANdleConfiguration();

    public Candle() {
        config.brightnessScalar = 0.5;
        candleLeft.configAllSettings(config);
        candleRight.configAllSettings(config);
    }
    public void setLEDs(int R, int G, int B){
        candleLeft.setLEDs(R,G,B);
        candleRight.setLEDs(R,G,B);
    }
    public void rainbow() {
        candleLeft.animate(rainbowAnim);
        candleRight.animate(rainbowAnim);

    }
    public void twinkle() {
        candleLeft.animate(twinkleAnim);
        candleRight.animate(twinkleAnim);
    }
}
