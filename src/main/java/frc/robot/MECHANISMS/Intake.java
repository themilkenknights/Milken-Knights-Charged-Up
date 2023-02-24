// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MECHANISMS;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.MISC.Motor;
import frc.robot.MISC.Constants.CANID;
import frc.robot.MISC.Constants.MKINTAKE;

// The Intake class contains everything relating to the intake mechanism
public class Intake {
  private Motor mMotor = Motor.getInstance();
  private TalonFX roller;
  private Solenoid intake;

  private Intake() {
    roller =
        mMotor.motor(
            CANID.rollerCANID, MKINTAKE.rollerNeutralMode, 0, MKINTAKE.pidf, MKINTAKE.inverted);

    intake = new Solenoid(CANID.revphCANID, PneumaticsModuleType.REVPH, CANID.intakePORT);
  }

  public static Intake getInstance() {
    return InstanceHolder.mInstance;
  }

  public void rollerSet(double setpoint) {
    roller.set(ControlMode.PercentOutput, setpoint);
  }

  public void toggle() {
    intake.toggle();
  }

  public void extend() {
    intake.set(true);
  }

  public void retract() {
    intake.set(false);
  }

  private static class InstanceHolder {
    private static final Intake mInstance = new Intake();
  }
}
