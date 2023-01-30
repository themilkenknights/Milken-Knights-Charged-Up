// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.MKRAMP;

/** Add your docs here. */
public class Ramp {

  private PIDController rampPID;

  public static Ramp getInstance() {
    return InstanceHolder.mInstance;
  }

  private Ramp() {
    rampPID = new PIDController(MKRAMP.kP, MKRAMP.kI, MKRAMP.kD);
  }

  public void rampMove() {
    double pitch = navx.getInstance().getNavxPitch();
    double pid = rampPID.calculate(pitch, 0);
    MkSwerveTrain.getInstance().etherSwerve(pid, 0, 0, ControlMode.PercentOutput);
  }

  private static class InstanceHolder {
    private static final Ramp mInstance = new Ramp();
  }
}
