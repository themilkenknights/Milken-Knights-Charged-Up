// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MISC;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.MISC.Constants.MKCANCODER;
import frc.robot.MISC.Constants.MKDRIVE;
import frc.robot.MISC.Constants.MKFALCON;
import frc.robot.MISC.Constants.MKTURN;

/** Add your docs here. */
public class Motor {

  private Motor() {
  }

  public static Motor getInstance() {
    return InstanceHolder.mInstance;
  }

  public TalonFX turnMotor(int canid) {
    TalonFX turn = new TalonFX(canid, "train");
    turn.configFactoryDefault();
    turn.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    turn.setNeutralMode(MKTURN.mode);
    turn.config_kP(0, MKTURN.kP);
    turn.config_kI(0, MKTURN.kI);
    turn.config_kD(0, MKTURN.kD);
    turn.config_kF(0, MKTURN.kF);
    turn.setInverted(MKTURN.inverted);
    turn.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_100Ms);
    turn.configVelocityMeasurementWindow(MKFALCON.velocityMeasAmount);
    turn.configVoltageCompSaturation(MKFALCON.voltComp);
    turn.enableVoltageCompensation(true);
    turn.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, MKFALCON.statusOneMeas);
    turn.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, MKFALCON.statusTwoMeas);
    turn.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 35, 60, 0.1));
    // true, const currlimit, peak limit, how many sec
    turn.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 60, 0.1));

    return turn;
  }

  public TalonFX driveMotor(int canid) {
    TalonFX drive = new TalonFX(canid, "train");
    drive.configFactoryDefault();
    drive.configOpenloopRamp(.3);
    drive.configClosedloopRamp(.3);
    drive.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    drive.setNeutralMode(MKDRIVE.mode);
    drive.config_kP(0, MKDRIVE.kP);
    drive.config_kI(0, MKDRIVE.kI);
    drive.config_kD(0, MKDRIVE.kD);
    drive.config_kF(0, MKDRIVE.kF);
    drive.setInverted(MKDRIVE.inverted);
    drive.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_100Ms);
    drive.configVelocityMeasurementWindow(MKFALCON.velocityMeasAmount);
    drive.configVoltageCompSaturation(MKFALCON.voltComp);
    drive.enableVoltageCompensation(true);
    drive.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, MKFALCON.statusOneMeas);
    drive.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, MKFALCON.statusTwoMeas);
    drive.configMotionCruiseVelocity(MKDRIVE.maxNativeVelocity);
    drive.configMotionAcceleration(MKDRIVE.maxNativeAcceleration);
    drive.configMotionSCurveStrength(MKDRIVE.scurve);
    drive.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 35, 60, 0.5));
    drive.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 60, 0.5));

    return drive;
  }

  public TalonFX motor(int canid, NeutralMode mode, int pidslot, double[] pidf, boolean inverted) {
    TalonFX motor = new TalonFX(canid, "rio");
    motor.configFactoryDefault();
    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    motor.setNeutralMode(mode);
    motor.config_kP(pidslot, pidf[0]);
    motor.config_kI(pidslot, pidf[1]);
    motor.config_kD(pidslot, pidf[2]);
    motor.config_kF(pidslot, pidf[3]);
    motor.setInverted(inverted);
    motor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_100Ms);
    motor.configVelocityMeasurementWindow(MKFALCON.velocityMeasAmount);
    motor.configVoltageCompSaturation(11);
    motor.enableVoltageCompensation(true);
    motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 35, 60, 0.5));
    motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 60, 0.5));
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, MKFALCON.statusOneMeas);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, MKFALCON.statusTwoMeas);
    return motor;
  }

  public TalonFX intake(int canid, NeutralMode mode, int pidslot, double[] pidf, boolean inverted) {
    TalonFX intake = new TalonFX(canid, "rio");
    intake.configFactoryDefault();
    intake.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    intake.setNeutralMode(mode);
    intake.config_kP(pidslot, pidf[0]);
    intake.config_kI(pidslot, pidf[1]);
    intake.config_kD(pidslot, pidf[2]);
    intake.config_kF(pidslot, pidf[3]);
    intake.setInverted(inverted);
    intake.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_100Ms);
    intake.configVelocityMeasurementWindow(MKFALCON.velocityMeasAmount);
    intake.configVoltageCompSaturation(11);
    intake.enableVoltageCompensation(true);
    // intake.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 35,
    // 60, 0.1));
    // intake.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35,
    // 60, 0.1));
    intake.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, MKFALCON.statusOneMeas);
    intake.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, MKFALCON.statusTwoMeas);
    return intake;
  }

  public CANCoder cancoder(int canid, double offset, String bus) {
    CANCoder encoder = new CANCoder(canid, bus);
    encoder.configFactoryDefault();
    encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    encoder.configAbsoluteSensorRange(MKCANCODER.range);
    encoder.configSensorDirection(MKCANCODER.inverted);
    encoder.configMagnetOffset(offset);
    // encoder.setPositionToAbsolute();

    return encoder;
  }

  public CANSparkMax Sparky(int canid, CANSparkMax.IdleMode Mode) {
    CANSparkMax sparky = new CANSparkMax(canid, MotorType.kBrushless);
    sparky.restoreFactoryDefaults();
    sparky.setIdleMode(Mode);
    sparky.setSmartCurrentLimit(20, 60, 5700);
    sparky.enableVoltageCompensation(11);
    return sparky;
  };

  private static class InstanceHolder {
    private static final Motor mInstance = new Motor();
  }
}
