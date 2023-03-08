// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CAMERA;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** The UltraSensor class contains everything relating to the ultrasonic sensor */
public class UltraSonic {
  private final AnalogInput ultrasonic = new AnalogInput(0);
  private double rawValue, voltageFactor, currentDistanceInches;

  public static UltraSonic getInstance() {
    return InstanceHolder.mInstance;
  }

  public void updateUltra() {
    rawValue = ultrasonic.getValue();
    voltageFactor = 5 / RobotController.getVoltage5V();
    currentDistanceInches = rawValue * voltageFactor * 0.125;
  }

  public void ultraSmartDashboard() {
    SmartDashboard.putNumber("inchesrounded", Math.round(currentDistanceInches));
    SmartDashboard.putNumber("rawvalue", rawValue);
    SmartDashboard.putNumber("inchesdistthingy", currentDistanceInches);
  }

  private static class InstanceHolder {
    private static final UltraSonic mInstance = new UltraSonic();
  }
}
