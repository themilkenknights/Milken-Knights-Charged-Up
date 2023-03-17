// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MECHANISMS;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.MISC.Constants.CANID;
import frc.robot.MISC.Constants.MKINTAKE;
import frc.robot.MISC.Motor;

// The Intake class contains everything relating to the intake mechanism
public class armintake {
  private Motor mMotor = Motor.getInstance();
  private TalonFX wristroller;
  private Solenoid intake;

  private armintake() {
    wristroller =
        mMotor.intake(
            CANID.wristroller, MKINTAKE.rollerNeutralMode, 0, MKINTAKE.pidf, MKINTAKE.inverted);

  }

  public static armintake getInstance() {
    return InstanceHolder.mInstance;
  }

  public void wristrollerset(double setpoint) {
    wristroller.set(ControlMode.PercentOutput, setpoint);
  }

  private static class InstanceHolder {
    private static final armintake mInstance = new armintake();
  }
}
