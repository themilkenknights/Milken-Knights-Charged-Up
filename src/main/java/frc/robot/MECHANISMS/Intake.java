// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MECHANISMS;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.MISC.Motor;
import frc.robot.MISC.Constants.CANID;
import frc.robot.MISC.Constants.MKINTAKE;

/** Add your docs here. */
public class Intake {
    private Motor motor = Motor.getInstance();

    private PIDController bottomIntake;
    private PIDController topIntake;

    private DigitalInput bottomSwitch;
    private DigitalInput topSwitch;

    private ShuffleboardTab mTab = Shuffleboard.getTab("intake");

    public GenericEntry tlIntake = mTab.add("tlIntake", 0).withPosition(0, 0).getEntry();
    public GenericEntry trIntake = mTab.add("trIntake", 0).withPosition(1, 0).getEntry();
    public GenericEntry blIntake = mTab.add("blIntake", 0).withPosition(2, 0).getEntry();
    public GenericEntry brIntake = mTab.add("brIntake", 0).withPosition(3, 0).getEntry();

    public GenericEntry tloutput = mTab.add("tloutput", 0).withPosition(0, 1).getEntry();
    public GenericEntry troutput = mTab.add("troutput", 0).withPosition(1, 1).getEntry();
    public GenericEntry bloutput = mTab.add("bloutput", 0).withPosition(2, 1).getEntry();
    public GenericEntry broutput = mTab.add("broutput", 0).withPosition(3, 1).getEntry();

    private GenericEntry topPID = mTab.add("topPID", 0).withPosition(1, 2).getEntry();
    private GenericEntry bottomPID = mTab.add("bottomPID", 0).withPosition(2, 2).getEntry();

    private TalonFX topLeft;
    private TalonFX topRight;

    private TalonFX bottomLeft;
    private TalonFX bottomRight;
    // positive is up, negative is down
    // left inverted

    private Intake() {
        bottomLeft = motor.motor(CANID.bottomLeftIntakeCANID, MKINTAKE.intakeNeutralMode, 0, MKINTAKE.pidf,
                MKINTAKE.bottomLeftInverted, "rio");
        bottomRight = motor.motor(CANID.bottomRightIntakeCANID, MKINTAKE.intakeNeutralMode, 0, MKINTAKE.pidf,
                MKINTAKE.bottomRightInverted, "rio");
        topLeft = motor.motor(CANID.topLeftIntakeCANID, MKINTAKE.intakeNeutralMode, 0, MKINTAKE.pidf,
                MKINTAKE.topLeftInverted, "rio");
        topRight = motor.motor(CANID.topRightIntakeCANID, MKINTAKE.intakeNeutralMode, 0, MKINTAKE.pidf,
                MKINTAKE.topRightInverted, "rio");

        bottomIntake = new PIDController(MKINTAKE.kP, MKINTAKE.kI, MKINTAKE.kD);
        topIntake = new PIDController(MKINTAKE.kP, MKINTAKE.kI, MKINTAKE.kD);

        bottomSwitch = new DigitalInput(9);
        topSwitch = new DigitalInput(8);
    }   

    public static Intake getInstance() {
        return InstanceHolder.mInstance;
    }

    public void moveBottomIntakePercentOutput(double setpoint) {
        bottomLeft.set(ControlMode.PercentOutput, setpoint);
        bottomRight.set(ControlMode.PercentOutput, setpoint);
    }
    public void moveBottomIntakePID(double setpoint) {
        bottomLeft.set(ControlMode.PercentOutput, bottomIntake.calculate(getBottomPositionNative(), setpoint));
        bottomRight.set(ControlMode.PercentOutput, bottomIntake.calculate(getBottomPositionNative(), setpoint));
    }

    
/*public void pidArm(double setpoint) {
    moveArm(
        -arm.calculate(getArmDegrees(), setpoint)
            - (Math.signum(arm.calculate(getArmDegrees(), setpoint)) * armFF(setpoint)),
        -arm.calculate(getArmDegrees(), setpoint)
            - (Math.signum(arm.calculate(getArmDegrees(), setpoint)) * armFF(setpoint)));
  }
  // SmartDashboard.putNumber("getrightmotoroutput",
    // armRight.getMotorOutputPercent());
 */
    public void moveTopIntakePercentOutput(double setpoint) {
        topLeft.set(ControlMode.PercentOutput, setpoint);
        topRight.set(ControlMode.PercentOutput, setpoint);
    }

    public void moveTopIntakePID(double setpoint) {
        topLeft.set(ControlMode.PercentOutput, bottomIntake.calculate(getBottomPositionNative(), setpoint));
        topRight.set(ControlMode.PercentOutput, bottomIntake.calculate(getBottomPositionNative(), setpoint));
    }

    public void stopBottomIntakePercentOutput() {
        bottomLeft.set(ControlMode.PercentOutput, 0);
        bottomRight.set(ControlMode.PercentOutput, 0);
    }

    public void stopTopIntakePercentOutput() {
        topLeft.set(ControlMode.PercentOutput, 0);
        topRight.set(ControlMode.PercentOutput, 0);
    }

    public double getBottomLeftPositionNative() {
        return bottomLeft.getSelectedSensorPosition();
    }

    public double getBottomRightPositionNative() {
        return bottomRight.getSelectedSensorPosition();
    }

    public double getBottomPositionNative() {
        return (getBottomLeftPositionNative() + getBottomRightPositionNative()) / 2;
    }

    public boolean getBottomSwitchEnabled() {
        return bottomSwitch.get();
    }

    public double getTopLeftPositionNative() {
        return topLeft.getSelectedSensorPosition();
    }

    public double getTopRightPositionNative() {
        return topRight.getSelectedSensorPosition();
    }

    public double getTopPositionNative() {
        return (getTopLeftPositionNative() + getTopRightPositionNative()) / 2;
    }

    public boolean getTopSwitchEnabled() {
        return topSwitch.get();
    }

    public void setBottomLeftEncoder(double setpoint) {
        bottomLeft.setSelectedSensorPosition(setpoint);
    }

    public void setBottomRightEncoder(double setpoint) {
        bottomRight.setSelectedSensorPosition(setpoint);
    }

    public void setTopLeftEncoder(double setpoint) {
        topLeft.setSelectedSensorPosition(setpoint);
    }

    public void setTopRightEncoder(double setpoint) {
        topRight.setSelectedSensorPosition(setpoint);
    }

    public void updateIntake() {
        blIntake.setDouble(getBottomLeftPositionNative());
        brIntake.setDouble(getBottomRightPositionNative());

        bloutput.setDouble(bottomLeft.getMotorOutputPercent());
        broutput.setDouble(bottomRight.getMotorOutputPercent());

        bottomPID.setDouble(bottomIntake.calculate(getBottomPositionNative(), bottomIntake.getSetpoint()));
        topPID.setDouble(topIntake.calculate(getTopPositionNative(), topIntake.getSetpoint()));
    }

    private static class InstanceHolder {
        private static final Intake mInstance = new Intake();
    }

}
