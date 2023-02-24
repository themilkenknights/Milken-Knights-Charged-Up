// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MISC;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/** Handles all dt related activities */
public class DeltaAirlines {
  private int loopOverrunWarning;
  private int loopCounter;
  private double lastTime = Timer.getFPGATimestamp();
  private double dt = 0;

  public static DeltaAirlines getInstance() {
    return InstanceHolder.mInstance;
  }

  public void updateDeltaTime() {
    loopCounter++;
    double time = Timer.getFPGATimestamp();
    dt = (time - lastTime) * 1e3;
    // SmartDashboard.putNumber("Loop Dt", dt);
    lastTime = time;

    if (dt > 22) {
      loopOverrunWarning++;
    }

    if (loopCounter == 500) {
      if (loopOverrunWarning > 10) {
        Shuffleboard.addEventMarker(
            "Loop Time Over 22ms for more than 10 loops in the past 5 seconds.",
            EventImportance.kHigh);
        loopCounter = 0;
      }
    }
  }

  public double getDT() {
    return dt;
  }

  private static class InstanceHolder {
    private static final DeltaAirlines mInstance = new DeltaAirlines();
  }
}
