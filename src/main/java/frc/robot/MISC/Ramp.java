// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MISC;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.MECHANISMS.MkSwerveTrain;
import frc.robot.MISC.Constants.MKRAMP;

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
    double pitch = pigeon.getInstance().getPigRoll();
    double pid = rampPID.calculate(pitch, 0);
    MkSwerveTrain.getInstance().etherSwerve(pid, 0, 0, ControlMode.PercentOutput);
    if (Math.abs(pitch) < 0 + MKRAMP.threshold) {
      // TODO see if this good
      MkSwerveTrain.getInstance().setModuleTurn(45, -45, 45, -45);
    }
  }
  // Returns true when the command should end.

  public boolean isFinished() {
    // return angle - Arm.getInstance().getArmDegrees() < 1.5;
    return Math.abs(pigeon.getInstance().getPigPitch()) <= 1;
  }

  private static class InstanceHolder {
    private static final Ramp mInstance = new Ramp();
  }
}
