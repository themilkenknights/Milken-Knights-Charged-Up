

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* 
package frc.robot.MISC;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.MISC.navx;
import frc.robot.MISC.Constants.LIGHTS;

/**The Lights class contains everything relating to ergabled*/

/* 
public class Lights {
    private double navXRot = 0;
    private int offset;

    private AddressableLED LEDS = new AddressableLED(LIGHTS.PWMPORT);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(LIGHTS.bufferNum);
    private Timer timer = new Timer();

    private Lights()
    {
        LEDS.setLength(LIGHTS.bufferNum);
        LEDS.setData(buffer);
        LEDS.start();
    }

    public static Lights getInstance() {
        return InstanceHolder.mInstance;
      }
      {


      if(lightMode == 0) {
        // For every pixel
        for (var i = 0; i < LIGHTS.bufferNum; i++) 
        {   
                timer.start();
                if(i < (LIGHTS.bufferNum / 3))
                {
                    buffer.setRGB((i+offset)%LIGHTS.bufferNum, 255, 25, LIGHTS.MaxRGBValue);
                }
                else if(i >= (LIGHTS.bufferNum / 3) && i < ((2 * LIGHTS.bufferNum) / 3))
                {
                    buffer.setRGB((i+offset)%LIGHTS.bufferNum, LIGHTS.MaxRGBValue, LIGHTS.MaxRGBValue, LIGHTS.MaxRGBValue);
                }
                else
                {
                    buffer.setRGB((i+offset)% LIGHTS.bufferNum, LIGHTS.MaxRGBValue, 0, 0);
                }
                if(timer.get() > 0.08)
                {
                    offset = (offset + 1) % LIGHTS.bufferNum;
                    timer.reset();
                    LEDS.setData(buffer);
                }
            }
        }
        if(lightMode == 1)
        {
                // For every pixel
                for (var i = 0; i < LIGHTS.bufferNum; i++) 
                {   
                        timer.start();
                        if(i < (LIGHTS.bufferNum / 3))
                        {
                            buffer.setRGB((i+offset)%LIGHTS.bufferNum, 148, 0, LIGHTS.MaxRGBValue);
                        }
                        else if(i >= (LIGHTS.bufferNum / 3) && i < ((2 * LIGHTS.bufferNum) / 3))
                        {
                            buffer.setRGB((i+offset)%LIGHTS.bufferNum, LIGHTS.MaxRGBValue, LIGHTS.MaxRGBValue, LIGHTS.MaxRGBValue);
                        }
                        else
                        {
                            buffer.setRGB((i+offset)% LIGHTS.bufferNum, LIGHTS.MaxRGBValue, 0, 0);
                        }
                        if(timer.get() > 0.08)
                        {
                            offset = (offset + 1) % LIGHTS.bufferNum;
                            timer.reset();
                            LEDS.setData(buffer);
                        }
                    }
        }
        if(lightMode == 2){
            // For every pixel
            for (var i = 0; i < LIGHTS.bufferNum; i++) 
            {   
                    timer.start();
                    if(i < (LIGHTS.bufferNum / 3))
                    {
                        buffer.setRGB((i+offset)%LIGHTS.bufferNum, 0, 0, LIGHTS.MaxRGBValue);
                    }
                    else if(i >= (LIGHTS.bufferNum / 3) && i < ((2 * LIGHTS.bufferNum) / 3))
                    {
                        buffer.setRGB((i+offset)%LIGHTS.bufferNum, LIGHTS.MaxRGBValue, LIGHTS.MaxRGBValue, LIGHTS.MaxRGBValue);
                    }
                    else
                    {
                        buffer.setRGB((i+offset)% LIGHTS.bufferNum, LIGHTS.MaxRGBValue, 0, 0);
                    }
                    if(timer.get() > 0.08)
                    {
                        offset = (offset + 1) % LIGHTS.bufferNum;
                        timer.reset();
                        LEDS.setData(buffer);
                    }
                }
            }
            private static class InstanceHolder {
                private static final Lights mInstance = new Lights();
              }
      }
    }

    */