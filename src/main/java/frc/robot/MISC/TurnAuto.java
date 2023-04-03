// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MISC;

import frc.robot.MECHANISMS.MkSwerveTrain;

/** Add your docs here. */
public class TurnAuto {
  private double angle;

  public static TurnAuto getInstance() {
    return InstanceHolder.mInstance;
  }

  public void setTurnAuto(double angle) {
    this.angle = angle;
  }

  public void updateTurnAuto() {
    MkSwerveTrain.getInstance().setModuleTurn(angle, angle, angle, angle);
  }

  public boolean isFinished() {
    return Math.abs(Math.abs(MkSwerveTrain.getInstance().vars.avgDeg) - Math.abs(angle)) <= 0.1;
  }

  private static class InstanceHolder {
    private static final TurnAuto mInstance = new TurnAuto();
  }
}
