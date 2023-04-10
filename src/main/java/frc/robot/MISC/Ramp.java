// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MISC;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  public void rampMove(double angle) {
    double pitch = pigeon.getInstance().getPigRoll();
    double pid = rampPID.calculate(pitch, 0);
    SmartDashboard.putNumber("pitch", pitch);
    SmartDashboard.putNumber("pid", pid);
    MkSwerveTrain.getInstance().etherSwerve(MathFormulas.limit(pid, -0.5, 0.5), 0,0, ControlMode.PercentOutput);
    if (Math.abs(pitch) < MKRAMP.threshold) {
      // TODO see if this good
      MkSwerveTrain.getInstance().setModuleTurn(0, 0, 0, 0);
    }
  }

  
  // Returns true when the command should end.

  public boolean isFinished() {
    // return angle - Arm.getInstance().getArmDegrees() < 1.5;
    return Math.abs(pigeon.getInstance().getPigRoll()) <= 1;
  }

  private static class InstanceHolder {
    private static final Ramp mInstance = new Ramp();
  }
}
