// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MISC;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import frc.robot.MISC.Constants.CANID;
import frc.robot.MISC.Constants.PIGEON;

/** pigeon stuff */
public class pigeon {
  private WPI_Pigeon2 pig = new WPI_Pigeon2(CANID.pigeonCANID, "rio");

  public static pigeon getInstance() {
    return InstanceHolder.mInstance;
  }

  public void setPigeon() {
    pig.configFactoryDefault();
  }

  public double getPigYaw() {
    return pig.getYaw();
  }

  public double getPigRoll() {
    return pig.getRoll();
  }

  public double getPigPitch() {
    return pig.getPitch();
  }

  public Pigeon2 getPig() {
    return pig;
  }

  public void reset() {
    pig.setYaw(0);
  }

  public double getAngleOfTip() {
    return Math.atan2(getPigPitch(), getPigRoll());
  }

  private static class InstanceHolder {
    private static final pigeon mInstance = new pigeon();
  }
}
