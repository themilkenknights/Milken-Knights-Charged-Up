// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MECHANISMS;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.MISC.Motor;
import frc.robot.MISC.Constants.CANID;
import frc.robot.MISC.Constants.MKINTAKE;

/** Add your docs here. */
public class Intake {
    private Motor motor = Motor.getInstance();

    private PIDController bottomIntake;

    private ShuffleboardTab mTab = Shuffleboard.getTab("intake");

    private GenericEntry tlIntake = mTab.add("tlIntake", 0).withPosition(0, 0).getEntry();
    private GenericEntry trIntake = mTab.add("trIntake", 0).withPosition(1, 0).getEntry();
    private GenericEntry blIntake = mTab.add("blIntake", 0).withPosition(2, 0).getEntry();
    private GenericEntry brIntake = mTab.add("brIntake", 0).withPosition(3, 0).getEntry();

    private GenericEntry tloutput = mTab.add("tloutput", 0).withPosition(0, 1).getEntry();
    private GenericEntry troutput = mTab.add("troutput", 0).withPosition(1, 1).getEntry();
    private GenericEntry bloutput = mTab.add("bloutput", 0).withPosition(2, 1).getEntry();
    private GenericEntry broutput = mTab.add("broutput", 0).withPosition(3, 1).getEntry();

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
                MKINTAKE.bottomLeftInverted, "train");
        bottomRight = motor.motor(CANID.bottomRightIntakeCANID, MKINTAKE.intakeNeutralMode, 0, MKINTAKE.pidf,
                MKINTAKE.bottomRightInverted, "train");

        bottomIntake = new PIDController(MKINTAKE.kP, MKINTAKE.kI, MKINTAKE.kD);
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

    public void stopBottomIntakePercentOutput() {
        bottomLeft.set(ControlMode.PercentOutput, 0);
        bottomRight.set(ControlMode.PercentOutput, 0);
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

    public void setBottomLeftEncoder(double setpoint) {
        bottomLeft.setSelectedSensorPosition(setpoint);
    }

    public void setBottomRightEncoder(double setpoint) {
        bottomRight.setSelectedSensorPosition(setpoint);
    }

    public void updateIntake() {
        blIntake.setDouble(getBottomLeftPositionNative());
        brIntake.setDouble(getBottomRightPositionNative());

        bloutput.setDouble(bottomLeft.getMotorOutputPercent());
        broutput.setDouble(bottomRight.getMotorOutputPercent());

        bottomPID.setDouble(bottomIntake.calculate(getBottomPositionNative(), bottomIntake.getSetpoint()));
    }

    private static class InstanceHolder {
        private static final Intake mInstance = new Intake();
    }

}
