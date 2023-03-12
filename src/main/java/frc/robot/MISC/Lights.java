// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MISC;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.MISC.Constants.LIGHTS;

/** The Lights class contains everything relating to ergabled */
public class Lights {
  // navx vlaue thing and light offset values so i dont have to create a new variable every time the
  // function runs
  private double navXRot = 0;
  private int offset;

  private AddressableLED LEDS = new AddressableLED(LIGHTS.PWMPORT);
  // private AddressableLED LEDStwo = new AddressableLED(4);

  private AddressableLEDBuffer buffer = new AddressableLEDBuffer(LIGHTS.bufferNum);
  // private AddressableLEDBuffer buffertwo = new AddressableLEDBuffer(LIGHTS.bufferNum);

  private Timer timer = new Timer();

  private Lights() {
    LEDS.setLength(LIGHTS.bufferNum);
    LEDS.setData(buffer);
    LEDS.start();

    // LEDStwo.setLength(LIGHTS.bufferNum);
    // LEDStwo.setData(buffertwo);
    // LEDStwo.start();
  }

  public static Lights getInstance() {
    return InstanceHolder.mInstance;
  }

  public void CONE() {
    // For every pixel
    for (var i = 0; i < LIGHTS.bufferNum; i++) {
      timer.start();
      buffer.setRGB(i, 255 / 3, 255 / 3, 0);
      // buffertwo.setRGB(i, 0, 0, LIGHTS.MaxRGBValue);
      if (timer.get() > 0.08) {
        offset = (offset + 1) % LIGHTS.bufferNum;
        timer.reset();
        LEDS.setData(buffer);
        // LEDStwo.setData(buffertwo);
      }
    }
  }

  public void CUBE() {
    // For every pixel
    for (var i = 0; i < LIGHTS.bufferNum; i++) {
      timer.start();
      buffer.setRGB(i, 148 / 2, 0, 211 / 2);
      // buffertwo.setRGB(i, 0, 0, LIGHTS.MaxRGBValue);
      if (timer.get() > 0.08) {
        offset = (offset + 1) % LIGHTS.bufferNum;
        timer.reset();
        LEDS.setData(buffer);
        // LEDStwo.setData(buffertwo);
      }
    }
  }

  /**
   * displays voltage on the leds
   *
   * @param volts volt value from driver station
   */
  public void voltage(double volts) {
    for (int i = 0; i < LIGHTS.bufferNum; i++) {
      buffer.setRGB(i, (int) (((((volts - 11) / 13) * LIGHTS.MaxRGBValue)) / 2), 0, 0);
    }
    LEDS.setData(buffer);
  }

  public void off() {
    for (int i = 0; i < LIGHTS.bufferNum; i++) {
      buffer.setRGB(i, 0, 0, 0);
      // buffertwo.setRGB(i, 0, 0, 0);
    }
    LEDS.setData(buffer);
    // LEDStwo.setData(buffertwo);

  }

  public void teamMode(Alliance all) {
    if (all == Alliance.Blue) {
      for (int i = 0; i < LIGHTS.bufferNum; i++) {
        buffer.setRGB(i, 0, 0, LIGHTS.MaxRGBValue);
      }
      LEDS.setData(buffer);
    } else {
      for (int i = 0; i < LIGHTS.bufferNum; i++) {
        buffer.setRGB(i, LIGHTS.MaxRGBValue, 0, 0);
      }
      LEDS.setData(buffer);
    }
  }

  private static class InstanceHolder {
    private static final Lights mInstance = new Lights();
  }
}
