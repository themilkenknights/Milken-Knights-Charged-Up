// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants.CANID;

/** Add your docs here. */
public class Claw {
  private Solenoid claw;

  private Claw() {
    claw = new Solenoid(CANID.revphCANID, PneumaticsModuleType.REVPH, CANID.CLAWPORT);
  }

  public static Claw getInstance() {
    return InstanceHolder.mInstance;
  }

  public void toggle() {
    claw.toggle();
  }

  public void extend() {
    claw.set(true);
  }

  public void retract() {
    claw.set(false);
  }

  private static class InstanceHolder {
    private static final Claw mInstance = new Claw();
  }
}
