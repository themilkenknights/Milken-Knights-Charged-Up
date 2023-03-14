// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MISC;

import com.kauailabs.navx.frc.AHRS;
import frc.robot.MISC.Constants.NAVX;

/** Add your docs here. */
public class navx {
  private AHRS navx = new AHRS();

  public static navx getInstance() {
    return InstanceHolder.mInstance;
  }

  public double getNavxYaw() {
    return NAVX.offsetYaw - navx.getYaw();
  }

  public double getNavxRoll() {
    return NAVX.offsetRoll - navx.getRoll();
  }

  public double getNavxYawREAL() {
    return navx.getYaw();
  }

  public double getNavxPitch() {
    return NAVX.offsetPitch - navx.getPitch();
  }

  public double getNavxRoll() {
    return NAVX.offsetRoll - navx.getRoll();
  }

  public AHRS getNavx() {
    return navx;
  }

  public double getNavxAltitude() {
    return navx.getAltitude();
  }

  public void reset() {
    navx.zeroYaw();
  }

  public boolean isAltWorking() {
    return navx.isAltitudeValid();
  }

  public double getAngleOfTip()
  {
    return Math.atan2(getNavxPitch(), getNavxRoll());
  }

  private static class InstanceHolder {
    private static final navx mInstance = new navx();
  }
}
