// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MECHANISMS;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.MISC.Constants;
import frc.robot.MISC.Constants.AUTO;
import frc.robot.MISC.Constants.CANID;
import frc.robot.MISC.Constants.MKCANCODER;
import frc.robot.MISC.Constants.MKDRIVE;
import frc.robot.MISC.Constants.MKTRAIN;
import frc.robot.MISC.Constants.MKTURN;
import frc.robot.MISC.DeltaAirlines;
import frc.robot.MISC.MathFormulas;
import frc.robot.MISC.Motor;
import frc.robot.MISC.pigeon;
import java.util.Map;

/** The Swerve class contains everything relating to the swerve mechanism */
public class MkSwerveTrain {
  public variables vars;

  private TalonFX topTurnLeft;
  private TalonFX topTurnRight;
  private TalonFX bottomTurnLeft;
  private TalonFX bottomTurnRight;

  private TalonFX topDriveLeft;
  private TalonFX topDriveRight;
  private TalonFX bottomDriveLeft;
  private TalonFX bottomDriveRight;

  private CANCoder topLeftCoder;
  private CANCoder topRightCoder;
  private CANCoder bottomLeftCoder;
  private CANCoder bottomRightCoder;

  private PIDController turn;
  private PIDController anti;

  private Motor mMotor = Motor.getInstance();

  private ShuffleboardTab mTab = Shuffleboard.getTab("mkswervetrain");

  private GenericEntry tlcoder = mTab.add("tlcoder", 0).withPosition(0, 0).getEntry();
  private GenericEntry trcoder = mTab.add("trcoder", 0).withPosition(1, 0).getEntry();
  private GenericEntry blcoder = mTab.add("blcoder", 0).withPosition(2, 0).getEntry();
  private GenericEntry brcoder = mTab.add("brcoder", 0).withPosition(3, 0).getEntry();

  private GenericEntry tlnatty = mTab.add("tlnatty", 0).withPosition(0, 1).getEntry();
  private GenericEntry trnatty = mTab.add("trnatty", 0).withPosition(1, 1).getEntry();
  private GenericEntry blnatty = mTab.add("blnatty", 0).withPosition(2, 1).getEntry();
  private GenericEntry brnatty = mTab.add("brnatty", 0).withPosition(3, 1).getEntry();

  private GenericEntry tlerr = mTab.add("tlerr", 0)
      .withWidget(BuiltInWidgets.kGraph)
      .withProperties(Map.of("Lower bound", -10, "Upper bound", 10))
      .withPosition(0, 3)
      .withSize(2, 2)
      .getEntry();
  private GenericEntry trerr = mTab.add("trerr", 0)
      .withWidget(BuiltInWidgets.kGraph)
      .withProperties(Map.of("Lower bound", -10, "Upper bound", 10))
      .withPosition(2, 3)
      .withSize(2, 2)
      .getEntry();
  private GenericEntry blerr = mTab.add("blerr", 0)
      .withWidget(BuiltInWidgets.kGraph)
      .withProperties(Map.of("Lower bound", -10, "Upper bound", 10))
      .withPosition(4, 3)
      .withSize(2, 2)
      .getEntry();
  private GenericEntry brerr = mTab.add("brerr", 0)
      .withWidget(BuiltInWidgets.kGraph)
      .withProperties(Map.of("Lower bound", -10, "Upper bound", 10))
      .withPosition(6, 3)
      .withSize(2, 2)
      .getEntry();

  private GenericEntry tlset = mTab.add("tlset", 0).withPosition(0, 2).getEntry();
  private GenericEntry trset = mTab.add("trset", 0).withPosition(1, 2).getEntry();
  private GenericEntry blset = mTab.add("blset", 0).withPosition(2, 2).getEntry();
  private GenericEntry brset = mTab.add("brset", 0).withPosition(3, 2).getEntry();

  private GenericEntry yaw = mTab.add("yaw", 0).withPosition(5, 1).getEntry();

  private MkSwerveTrain() {
    vars = new variables();
    vars.hIntegral = 0;
    vars.mod1 = new double[2];
    vars.mod2 = new double[2];
    vars.mod3 = new double[2];
    vars.mod4 = new double[2];
    vars.pointOne = new double[] { 1, 10 };
    vars.pointTwo = new double[] { 1, 13 };

    turn = new PIDController(vars.hP, vars.hI, vars.hD);
    turn.enableContinuousInput(0, 360);

    anti = new PIDController(MKTRAIN.kP, MKTRAIN.kI, MKTRAIN.kD);

    topTurnLeft = mMotor.turnMotor(CANID.topTurnLeftCANID);
    topTurnRight = mMotor.turnMotor(CANID.topTurnRightCANID);
    bottomTurnLeft = mMotor.turnMotor(CANID.bottomTurnLeftCANID);
    bottomTurnRight = mMotor.turnMotor(CANID.bottomTurnRightCANID);

    topDriveLeft = mMotor.driveMotor(CANID.topDriveLeftCANID);
    topDriveRight = mMotor.driveMotor(CANID.topDriveRightCANID);
    bottomDriveLeft = mMotor.driveMotor(CANID.bottomDriveLeftCANID);
    bottomDriveRight = mMotor.driveMotor(CANID.bottomDriveRightCANID);

    topLeftCoder = mMotor.cancoder(CANID.topTurnLeftCANCoderCANID, MKCANCODER.topLeftOffset, "rio");
    topRightCoder = mMotor.cancoder(CANID.topTurnRightCANCoderCANID, MKCANCODER.topRightOffset, "rio");
    bottomLeftCoder = mMotor.cancoder(CANID.bottomTurnLeftCANCoderCANID, MKCANCODER.bottomLeftOffset, "rio");
    bottomRightCoder = mMotor.cancoder(CANID.bottomTurnRightCANCoderCANID, MKCANCODER.bottomRightOffset, "rio");
  }

  public static MkSwerveTrain getInstance() {
    return InstanceHolder.mInstance;
  }

  /** start turn and drive functions */
  public void startTrain() {
    vars.hIntegral = 0;
    startDrive();
    startTurn();
  }

  /** Set turn motors to cancoder position */
  public void startTurn() {
    topTurnLeft.setSelectedSensorPosition(
        MathFormulas.degreesToNative(topLeftCoder.getAbsolutePosition(), MKTURN.greerRatio));
    topTurnRight.setSelectedSensorPosition(
        MathFormulas.degreesToNative(topRightCoder.getAbsolutePosition(), MKTURN.greerRatio));
    bottomTurnLeft.setSelectedSensorPosition(
        MathFormulas.degreesToNative(bottomLeftCoder.getAbsolutePosition(), MKTURN.greerRatio));
    bottomTurnRight.setSelectedSensorPosition(
        MathFormulas.degreesToNative(bottomRightCoder.getAbsolutePosition(), MKTURN.greerRatio));
  }

  public void resetDrive() {
    topDriveLeft.setSelectedSensorPosition(0);
    topDriveRight.setSelectedSensorPosition(0);
    bottomDriveLeft.setSelectedSensorPosition(0);
    bottomDriveRight.setSelectedSensorPosition(0);
  }

  /** Zero drive */
  public void startDrive() {
    topDriveLeft.setSelectedSensorPosition(0);
    topDriveRight.setSelectedSensorPosition(0);
    bottomDriveLeft.setSelectedSensorPosition(0);
    bottomDriveRight.setSelectedSensorPosition(0);
  }

  public double tlDeg() {
    return MathFormulas.nativeToDegrees(topTurnLeft.getSelectedSensorPosition(), MKTURN.greerRatio);
  }

  public double trDeg() {
    return MathFormulas.nativeToDegrees(
        topTurnRight.getSelectedSensorPosition(), MKTURN.greerRatio);
  }

  public double blDeg() {
    return MathFormulas.nativeToDegrees(
        bottomTurnLeft.getSelectedSensorPosition(), MKTURN.greerRatio);
  }

  public double brDeg() {
    return MathFormulas.nativeToDegrees(
        bottomTurnRight.getSelectedSensorPosition(), MKTURN.greerRatio);
  }

  public double tlCoder() {
    return topLeftCoder.getAbsolutePosition();
  }

  public double trCoder() {
    return topRightCoder.getAbsolutePosition();
  }

  public double blCoder() {
    return bottomLeftCoder.getAbsolutePosition();
  }

  public double brCoder() {
    return bottomRightCoder.getAbsolutePosition();
  }

  public TalonFX tlMotor() {
    return topDriveLeft;
  }

  public TalonFX trMotor() {
    return topDriveRight;
  }

  public TalonFX blMotor() {
    return bottomDriveLeft;
  }

  public TalonFX brMotor() {
    return bottomDriveRight;
  }

  public TalonFX turnMotor(String motor) {
    motor = motor.toLowerCase();
    if (motor == "tl") {
      return topTurnLeft;
    } else if (motor == "tr") {
      return topTurnRight;
    } else if (motor == "bl") {
      return bottomTurnLeft;
    } else if (motor == "br") {
      return bottomTurnRight;
    } else {
      return null;
    }
  }

  public SwerveModulePosition modulePosTL() {
    return new SwerveModulePosition(
        MathFormulas.nativeToMeters(topDriveLeft.getSelectedSensorPosition()),
        Rotation2d.fromDegrees(tlDeg() + 180));
  }

  public SwerveModulePosition modulePosTR() {
    return new SwerveModulePosition(
        MathFormulas.nativeToMeters(topDriveRight.getSelectedSensorPosition()),
        Rotation2d.fromDegrees(trDeg() + 180));
  }

  public SwerveModulePosition modulePosBL() {
    return new SwerveModulePosition(
        MathFormulas.nativeToMeters(bottomDriveLeft.getSelectedSensorPosition()),
        Rotation2d.fromDegrees(blDeg() + 180));
  }

  public SwerveModulePosition modulePosBR() {
    return new SwerveModulePosition(
        MathFormulas.nativeToMeters(bottomDriveRight.getSelectedSensorPosition()),
        Rotation2d.fromDegrees(brDeg() + 180));
  }

  public void updateSwerve() {
    SmartDashboard.putNumber("roll", pigeon.getInstance().getPigRoll());
    DeltaAirlines.getInstance().updateDeltaTime();

    // SmartDashboard.putNumber("distancetopright", vars.posInchTR);
    // AUTO.measToPredictRatio);
    // SmartDashboard.putNumber("avgDistinches", vars.avgDistInches);
    // SmartDashboard.putNumber("mod1", vars.mod1[1]);
    // SmartDashboard.putNumber("mod2", vars.mod1[1]);
    // SmartDashboard.putNumber("mod3", vars.mod1[1]);
    // SmartDashboard.putNumber("mod4", vars.mod1[1]);

    vars.yaw = pigeon.getInstance().getPigYaw() % 360;

    tlcoder.setDouble(((tlCoder() % 360) + 360) % 360);
    trcoder.setDouble(((trCoder() % 360) + 360) % 360);
    blcoder.setDouble(((blCoder() % 360) + 360) % 360);
    brcoder.setDouble(((brCoder() % 360) + 360) % 360);

    tlset.setDouble(((vars.mod2[1] % 360) + 360) % 360);
    trset.setDouble(((vars.mod1[1] % 360) + 360) % 360);
    blset.setDouble(((vars.mod4[1] % 360) + 360) % 360);
    brset.setDouble(((vars.mod3[1] % 360) + 360) % 360);

    tlnatty.setDouble(
        ((MathFormulas.nativeToDegrees(topTurnLeft.getSelectedSensorPosition(), MKTURN.greerRatio)
            % 360)
            + 360)
            % 360);
    trnatty.setDouble(
        ((MathFormulas.nativeToDegrees(topTurnRight.getSelectedSensorPosition(), MKTURN.greerRatio)
            % 360)
            + 360)
            % 360);
    blnatty.setDouble(
        ((MathFormulas.nativeToDegrees(
            bottomTurnLeft.getSelectedSensorPosition(), MKTURN.greerRatio)
            % 360)
            + 360)
            % 360);
    brnatty.setDouble(
        ((MathFormulas.nativeToDegrees(
            bottomTurnRight.getSelectedSensorPosition(), MKTURN.greerRatio)
            % 360)
            + 360)
            % 360);

    tlerr.setDouble(
        Math.abs(
            ((MathFormulas.nativeToDegrees(
                topTurnLeft.getSelectedSensorPosition(), MKTURN.greerRatio)
                % 360)
                + 360)
                % 360)
            - ((vars.mod2[1] % 360) + 360) % 360);
    trerr.setDouble(
        Math.abs(
            ((MathFormulas.nativeToDegrees(
                topTurnRight.getSelectedSensorPosition(), MKTURN.greerRatio)
                % 360)
                + 360)
                % 360)
            - ((vars.mod1[1] % 360) + 360) % 360);
    blerr.setDouble(
        Math.abs(
            ((MathFormulas.nativeToDegrees(
                bottomTurnLeft.getSelectedSensorPosition(), MKTURN.greerRatio)
                % 360)
                + 360)
                % 360)
            - ((vars.mod4[1] % 360) + 360) % 360);
    brerr.setDouble(
        Math.abs(
            ((MathFormulas.nativeToDegrees(
                bottomTurnRight.getSelectedSensorPosition(), MKTURN.greerRatio)
                % 360)
                + 360)
                % 360)
            - ((vars.mod3[1] % 360) + 360) % 360);

    yaw.setDouble(vars.yaw);

    /*
     * SmartDashboard.putNumber("topleftnativeDRIVE",
     * (topDriveLeft.getSelectedSensorVelocity()));
     * SmartDashboard.putNumber("toprightnativeDRIVE",
     * (topDriveRight.getSelectedSensorVelocity()));
     * SmartDashboard.putNumber("bottomleftnativeDRIVE",
     * (bottomDriveLeft.getSelectedSensorVelocity()));
     * SmartDashboard.putNumber("bottomrightnativeDRIVE",
     * (bottomDriveRight.getSelectedSensorVelocity()));
     */
    MathFormulas.nativeToDegrees(topTurnLeft.getSelectedSensorPosition(), MKTURN.greerRatio);

    // SmartDashboard.putNumber("topturnleftmotorposnative",
    // topTurnLeft.getSelectedSensorPosition());
    // SmartDashboard.putNumber("ticksforoffset", MathFormulas.nativeToDegrees(217,
    // MKTURN.greerRatio));
    // SmartDashboard.putNumber("magicangle", vars.magicAngle);
    // SmartDashboard.putNumber("autp dist test", vars.avgDistTest);
    // SmartDashboard.putNumber("total", vars.totalDistance);
    // SmartDashboard.putBoolean("idsone", isMotionMagicDone());
    // SmartDashboard.putNumber("avgvelinches", vars.avgVelInches);
    // SmartDashboard.putNumber("avgdist", vars.avgDistInches);

    vars.posInchTL = MathFormulas.nativeToInches(topDriveLeft.getSelectedSensorPosition());
    vars.posInchTR = MathFormulas.nativeToInches(topDriveRight.getSelectedSensorPosition());
    vars.posInchBL = MathFormulas.nativeToInches(bottomDriveLeft.getSelectedSensorPosition());
    vars.posInchBR = MathFormulas.nativeToInches(bottomDriveRight.getSelectedSensorPosition());

    vars.velNativeTL = topDriveLeft.getSelectedSensorVelocity();
    vars.velNativeTR = topDriveRight.getSelectedSensorVelocity();
    vars.velNativeBL = bottomDriveLeft.getSelectedSensorVelocity();
    vars.velNativeBR = bottomDriveRight.getSelectedSensorVelocity();

    vars.velInchTL = MathFormulas.nativePer100MstoInchesPerSec(vars.velNativeTL);
    vars.velInchTR = MathFormulas.nativePer100MstoInchesPerSec(vars.velNativeTR);
    vars.velInchBL = MathFormulas.nativePer100MstoInchesPerSec(vars.velNativeBL);
    vars.velInchBR = MathFormulas.nativePer100MstoInchesPerSec(vars.velNativeBR);

    vars.degTL = tlDeg();
    vars.degTR = trDeg();
    vars.degBL = blDeg();
    vars.degBR = brDeg();

    vars.avgDistInches = (Math.abs(vars.posInchTL)
        + Math.abs(vars.posInchTR)
        + Math.abs(vars.posInchBL)
        + Math.abs(vars.posInchBR))
        / 4.0;
    vars.avgVelInches = (vars.velInchTL + vars.velInchTR + vars.velInchBL + vars.velInchBR) / 4.0;
    vars.avgVelNative = (vars.velNativeTL + vars.velNativeTR + vars.velNativeBL + vars.velNativeBR) / 4.0;
    vars.avgDeg = (vars.degTL + vars.degTR + vars.degBL + vars.degBR) / 4.0;
  }

  public void setModuleTurn(double tl, double tr, double bl, double br) {
    topTurnLeft.set(ControlMode.Position, MathFormulas.degreesToNative(tl, MKTURN.greerRatio));
    topTurnRight.set(ControlMode.Position, MathFormulas.degreesToNative(tr, MKTURN.greerRatio));
    bottomTurnLeft.set(ControlMode.Position, MathFormulas.degreesToNative(bl, MKTURN.greerRatio));
    bottomTurnRight.set(ControlMode.Position, MathFormulas.degreesToNative(br, MKTURN.greerRatio));
  }

  public void setModuleDrive(ControlMode mode, double tl, double tr, double bl, double br) {
    topDriveLeft.set(mode, tl);
    topDriveRight.set(mode, tr);
    bottomDriveLeft.set(mode, bl); // Ben
    bottomDriveRight.set(mode, br);
  }

  public void stopEverything() {
    topTurnLeft.set(ControlMode.PercentOutput, 0);
    topTurnRight.set(ControlMode.PercentOutput, 0);
    bottomTurnLeft.set(ControlMode.PercentOutput, 0);
    bottomTurnRight.set(ControlMode.PercentOutput, 0);

    topDriveLeft.set(ControlMode.PercentOutput, 0);
    topDriveRight.set(ControlMode.PercentOutput, 0);
    bottomDriveLeft.set(ControlMode.PercentOutput, 0);
    bottomDriveRight.set(ControlMode.PercentOutput, 0);
  }

  /**
   * See <a href=
   * "https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383">this
   * thread</a> for more information.
   *
   * <p>
   * Note - this function uses 180 minus yaw due to the positioning of our pigeon.
   *
   * @param FWD Forward axis of controller
   * @param STR Strafe axis of controller
   * @param RCW Rotational axis of controller
   * @author Ether
   */
  public void etherSwerve(double FWD, double STR, double RCW, ControlMode mode) {
    vars.temp = FWD * Math.cos(Math.toRadians(vars.yaw)) + STR * Math.sin(Math.toRadians(vars.yaw));
    STR = -FWD * Math.sin(Math.toRadians(vars.yaw)) + STR * Math.cos(Math.toRadians(vars.yaw));
    FWD = vars.temp;

    vars.A = STR - RCW * (MKTRAIN.L / MKTRAIN.R);
    vars.B = STR + RCW * (MKTRAIN.L / MKTRAIN.R);
    vars.C = FWD - RCW * (MKTRAIN.W / MKTRAIN.R);
    vars.D = FWD + RCW * (MKTRAIN.W / MKTRAIN.R);

    // SmartDashboard.putNumber("FWD", FWD);
    // SmartDashboard.putNumber("STR", STR);

    vars.mod2[1] = Math.atan2(vars.B, vars.C) * 180.0 / Constants.kPi;
    vars.mod1[1] = Math.atan2(vars.B, vars.D) * 180.0 / Constants.kPi;
    vars.mod3[1] = Math.atan2(vars.A, vars.D) * 180.0 / Constants.kPi;
    vars.mod4[1] = Math.atan2(vars.A, vars.C) * 180.0 / Constants.kPi;

    vars.mod2[0] = Math.sqrt((Math.pow(vars.B, 2.0)) + (Math.pow(vars.C, 2.0)));
    vars.mod1[0] = Math.sqrt((Math.pow(vars.B, 2.0)) + (Math.pow(vars.D, 2.0)));
    vars.mod3[0] = Math.sqrt((Math.pow(vars.A, 2.0)) + (Math.pow(vars.D, 2.0)));
    vars.mod4[0] = Math.sqrt((Math.pow(vars.A, 2.0)) + (Math.pow(vars.C, 2.0)));

    vars.max = vars.mod1[0];
    if (vars.mod2[0] > vars.max)
      vars.max = vars.mod2[0];
    if (vars.mod3[0] > vars.max)
      vars.max = vars.mod3[0];
    if (vars.mod4[0] > vars.max)
      vars.max = vars.mod4[0];
    if (vars.max > 1.0) {
      vars.mod1[0] /= vars.max;
      vars.mod2[0] /= vars.max;
      vars.mod3[0] /= vars.max;
      vars.mod4[0] /= vars.max;
    }

    vars.mod2 = setDirection(tlDeg(), vars.mod2);
    vars.mod1 = setDirection(trDeg(), vars.mod1);
    vars.mod4 = setDirection(blDeg(), vars.mod4);
    vars.mod3 = setDirection(brDeg(), vars.mod3);

    if (mode == ControlMode.MotionMagic) {
      vars.mod1[0] = Math.signum(vars.mod1[0]) * vars.autoDist;
      vars.mod2[0] = Math.signum(vars.mod2[0]) * vars.autoDist;
      vars.mod3[0] = Math.signum(vars.mod3[0]) * vars.autoDist;
      vars.mod4[0] = Math.signum(vars.mod4[0]) * vars.autoDist;
    }

    // etherRCWFinder(FWD, STR, RCW);
    setModuleDrive(mode, vars.mod2[0], vars.mod1[0], vars.mod4[0], vars.mod3[0]);
    setModuleTurn(vars.mod2[1], vars.mod1[1], vars.mod4[1], vars.mod3[1]);
  }

  public void resetRCWFinder() {
    vars.ATest = 0;
    vars.BTest = 0;
    vars.CTest = 0;
    vars.DTest = 0;

    vars.mod1Test = 0;
    vars.mod2Test = 0;
    vars.mod3Test = 0;
    vars.mod4Test = 0;
  }

  public void etherRCWFinder(double FWD, double STR, double RCW) {
    vars.dt = DeltaAirlines.getInstance().getDT();
    if (vars.dt > 21) {
      vars.dt = 20;
    }
    vars.yawTest = 0.0;
    vars.tempTest = FWD * Math.cos(Math.toRadians(vars.yawTest)) + STR * Math.sin(Math.toRadians(vars.yawTest));
    STR = -FWD * Math.sin(Math.toRadians(vars.yawTest))
        + STR * Math.cos(Math.toRadians(vars.yawTest));
    FWD = vars.tempTest;

    // System.out.println("FWD" + FWD);
    // System.out.println("STR" + STR);
    // System.out.println("RCW" + RCW);

    // SmartDashboard.putNumber("FWD", FWD);
    // SmartDashboard.putNumber("STR", STR);

    vars.ATest = STR - RCW * (MKTRAIN.L / MKTRAIN.R);
    vars.BTest = STR + RCW * (MKTRAIN.L / MKTRAIN.R);
    vars.CTest = FWD - RCW * (MKTRAIN.W / MKTRAIN.R);
    vars.DTest = FWD + RCW * (MKTRAIN.W / MKTRAIN.R);

    // System.out.println("mod2test1" + vars.mod2Test);

    vars.mod2Test = (Math.sqrt((Math.pow(vars.BTest, 2.0)) + (Math.pow(vars.CTest, 2.0))));
    vars.mod1Test = (Math.sqrt((Math.pow(vars.BTest, 2.0)) + (Math.pow(vars.DTest, 2.0))));
    vars.mod3Test = (Math.sqrt((Math.pow(vars.ATest, 2.0)) + (Math.pow(vars.DTest, 2.0))));
    vars.mod4Test = (Math.sqrt((Math.pow(vars.ATest, 2.0)) + (Math.pow(vars.CTest, 2.0))));

    // System.out.println("mod2test2" + vars.mod2Test);

    vars.maxTest = vars.mod1Test;
    if (vars.mod2Test > vars.maxTest)
      vars.maxTest = vars.mod2Test;
    if (vars.mod3Test > vars.maxTest)
      vars.maxTest = vars.mod3Test;
    if (vars.mod4Test > vars.maxTest)
      vars.maxTest = vars.mod4Test;
    if (vars.maxTest > 1.0) {
      vars.mod1Test /= vars.maxTest;
      vars.mod2Test /= vars.maxTest;
      vars.mod3Test /= vars.maxTest;
      vars.mod4Test /= vars.maxTest;
    }

    // System.out.println("mod2test3" + vars.mod2Test);
    // System.out.println("dt" + vars.dt);
    vars.mod2Test = MathFormulas.nativePer100MsToInches(MKDRIVE.maxNativeVelocity * vars.mod2Test, vars.dt);
    vars.mod1Test = MathFormulas.nativePer100MsToInches(MKDRIVE.maxNativeVelocity * vars.mod1Test, vars.dt);
    vars.mod3Test = MathFormulas.nativePer100MsToInches(MKDRIVE.maxNativeVelocity * vars.mod3Test, vars.dt);
    vars.mod4Test = MathFormulas.nativePer100MsToInches(MKDRIVE.maxNativeVelocity * vars.mod4Test, vars.dt);
    // System.out.println("dt" + vars.dt);
    // System.out.println("mod2test4" + vars.mod2Test);

    vars.avgDistTest = (vars.avgDistTest
        + ((Math.abs(vars.mod1Test)
            + Math.abs(vars.mod2Test)
            + Math.abs(vars.mod3Test)
            + Math.abs(vars.mod4Test))
            / 4.0));
  }

  public void etherAutoSwerve(double FWD, double STR, double RCW, ControlMode mode) {
    etherRCWFinder(FWD, STR, 0);
    vars.temp = FWD * Math.cos(Math.toRadians(vars.yaw)) + STR * Math.sin(Math.toRadians(vars.yaw));
    STR = -FWD * Math.sin(Math.toRadians(vars.yaw)) + STR * Math.cos(Math.toRadians(vars.yaw));
    FWD = vars.temp;

    // SmartDashboard.putNumber("FWDreal", FWD);
    // SmartDashboard.putNumber("STRreal", STR);
    //SmartDashboard.putNumber("RCWreal", RCW);

    vars.A = STR - RCW * (MKTRAIN.L / MKTRAIN.R);
    vars.B = STR + RCW * (MKTRAIN.L / MKTRAIN.R);
    vars.C = FWD - RCW * (MKTRAIN.W / MKTRAIN.R);
    vars.D = FWD + RCW * (MKTRAIN.W / MKTRAIN.R);

    vars.mod2[1] = Math.atan2(vars.B, vars.C) * 180.0 / Constants.kPi;
    vars.mod1[1] = Math.atan2(vars.B, vars.D) * 180.0 / Constants.kPi;
    vars.mod3[1] = Math.atan2(vars.A, vars.D) * 180.0 / Constants.kPi;
    vars.mod4[1] = Math.atan2(vars.A, vars.C) * 180.0 / Constants.kPi;

    vars.mod2[0] = Math.sqrt((Math.pow(vars.B, 2.0)) + (Math.pow(vars.C, 2.0)));
    vars.mod1[0] = Math.sqrt((Math.pow(vars.B, 2.0)) + (Math.pow(vars.D, 2.0)));
    vars.mod3[0] = Math.sqrt((Math.pow(vars.A, 2.0)) + (Math.pow(vars.D, 2.0)));
    vars.mod4[0] = Math.sqrt((Math.pow(vars.A, 2.0)) + (Math.pow(vars.C, 2.0)));

    vars.max = vars.mod1[0];
    if (vars.mod2[0] > vars.max)
      vars.max = vars.mod2[0];
    if (vars.mod3[0] > vars.max)
      vars.max = vars.mod3[0];
    if (vars.mod4[0] > vars.max)
      vars.max = vars.mod4[0];
    if (vars.max > 1) {
      vars.mod1[0] /= vars.max;
      vars.mod2[0] /= vars.max;
      vars.mod3[0] /= vars.max;
      vars.mod4[0] /= vars.max;
    }

    vars.mod2 = setDirection(tlDeg(), vars.mod2);
    vars.mod1 = setDirection(trDeg(), vars.mod1);
    vars.mod4 = setDirection(blDeg(), vars.mod4);
    vars.mod3 = setDirection(brDeg(), vars.mod3);

    // SmartDashboard.putNumber("a", vars.A);
    // SmartDashboard.putNumber("b", vars.B);
    // SmartDashboard.putNumber("c", vars.C);
    // SmartDashboard.putNumber("d", vars.D);
    // SmartDashboard.putNumber("topdrileftvelo",
    // topDriveLeft.getSelectedSensorVelocity());
    // SmartDashboard.putNumber("mod1[0]", vars.mod1[0]);
    setModuleDrive(mode, vars.mod2[0], vars.mod1[0], vars.mod4[0], vars.mod3[0]);
    setModuleTurn(vars.mod2[1], vars.mod1[1], vars.mod4[1], vars.mod3[1]);
  }

  /** move robot to angle/heading */
  public double moveToAngy(double set) {
    /*
     * vars.hError = set - Math.abs(vars.yaw); // Error = Target - Actual
     * vars.hIntegral += (vars.hError
     * .02); // Integral is increased by the error*time (which is .02 seconds using
     * normal
     * // IterativeRobot)
     * vars.hDerivative = (vars.hError - vars.hPreviousError) / .02;
     * vars.hPreviousError = vars.hError;
     * return vars.hP * vars.hError + vars.hI * vars.hIntegral + vars.hD *
     * vars.hDerivative;
     */

    double setpoint = turn.calculate(
        Math.abs(((vars.yaw % 360) + 360) % 360), Math.abs(((set % 360) + 360) % 360));
    // SmartDashboard.putNumber("err for move to angy", turn.getPositionError());
    // SmartDashboard.putNumber("closest angle",
    // MathFormulas.closestAngle(Math.abs(((vars.yaw %
    // 360) + 360) % 360), Math.abs(((set % 360) + 360) % 360)));
    SmartDashboard.putNumber("yaw for move to angy", Math.abs(((vars.yaw % 360) + 360) % 360));
    SmartDashboard.putNumber("setpoint for move to angy", Math.abs(((set % 360) + 360) % 360));

    return -setpoint;
  }

  /**
   * decides whether a driving motor should flip based on where the angular
   * motor's setpoint is.
   *
   * @param position position of the motor
   * @param setpoint setpoint for the motor
   * @return returns best angle of travel for the angular motor, as well as the
   *         flip value for the
   *         driving motor (as an array so it can return two things in one instead
   *         of two seperatly)
   * @author team 6624
   */
  public static double[] setDirection(double pos, double[] mod) {
    double currentAngle = pos;
    // find closest angle to setpoint
    double setpointAngle = MathFormulas.closestAngle(currentAngle, mod[1]);
    // find closest angle to setpoint + 180
    double setpointAngleFlipped = MathFormulas.closestAngle(currentAngle, mod[1] + 180.0);
    // if the closest angle to setpoint is shorter
    if (Math.abs(setpointAngle) <= Math.abs(setpointAngleFlipped)) {
      // unflip the motor direction use the setpoint
      return new double[] { mod[0], (currentAngle + setpointAngle) };
    }
    // if the closest angle to setpoint + 180 is shorter
    else {
      // flip the motor direction and use the setpoint + 180
      return new double[] { Math.abs(mod[0]) * -1, (currentAngle + setpointAngleFlipped) };
    }
  }

  /**
   * decides whether a driving motor should flip based on where the angular
   * motor's setpoint is.
   *
   * @param position position of the motor
   * @param setpoint setpoint for the motor
   * @return returns best angle of travel for the angular motor, as well as the
   *         flip value for the
   *         driving motor (as an array so it can return two things in one instead
   *         of two seperatly)
   * @author team 6624
   */
  public static double setDirectionAuto(double pos, double setpoint) {
    // find closest angle to setpoint
    if (MathFormulas.closestAngle(pos, setpoint) < 90) {
      // unflip the motor direction use the setpoint
      return setpoint;
    }
    // if the closest angle to setpoint + 180 is shorter
    else {
      // flip the motor direction and use the setpoint + 180
      return -(setpoint);
    }
  }

  public void setEtherAuto(double Distance, double disteA, double radius) {
    vars.autoDist = MathFormulas.inchesToNative(Distance);
    vars.totalDistance = Distance;
    vars.avgDistInches = 0;
    vars.distanceA = disteA;
    vars.avgDistTest = 0;
    // SmartDashboard.putNumber("totaldistancccee", vars.totalDistance);
  }

  /**
   * Using the {@link #swerveAutonomousEther} and motion magic, an autonomous
   * angled path of motion
   * can be achieved
   *
   * @param totalDistance Length of curved path
   * @param thetaTurn     Angle of curved path
   * @param RCWauto       [-1, 1] For spinny, 0 for no spinny
   * @param mode          Curve or Straight
   * @param turny         Specific or Infinite
   * @param turnyAuto     (if using specific for turny) angle that robot tries to
   *                      keep when moving
   */
  public void etherAutoUpdate(double thetaTurn, double heading, double side) {
    // numbers fall short of high by 3ish inches and short of length by 4ish inches
    double calcangle = side
        * ((heading)
            + (((-thetaTurn / 2.0)
                + (((vars.avgDistInches) / (vars.totalDistance)) * (thetaTurn)))));

    vars.RCWtemp = moveToAngy(180);
    vars.FWDauto = (-1 * Math.cos(calcangle * (Constants.kPi / 180.0))) / 5.0;
    vars.STRauto = (Math.sin(calcangle * (Constants.kPi / 180.0))) / 5.0;

    etherAutoSwerve(vars.FWDauto, vars.STRauto, vars.RCWtemp / 5.0, ControlMode.PercentOutput);
    etherRCWFinder(vars.FWDauto, vars.STRauto, 0.0);
    /*
     * SmartDashboard.putNumber(
     * "avgdistsimilarity", vars.avgDistInches - (vars.avgDistTest *
     * AUTO.measToPredictRatio));
     * SmartDashboard.putNumber(
     * "calcangletestsimilarrity",
     * ((heading)
     * + (((-thetaTurn / 2)
     * + (((vars.avgDistInches) / (vars.totalDistance)) * (thetaTurn)))))
     * % 360);
     * SmartDashboard.putNumber("FWDauto", vars.FWDauto);
     * SmartDashboard.putNumber("STRauto", vars.STRauto);
     * SmartDashboard.putNumber("calcangle", calcangle % 360);
     * SmartDashboard.putBoolean("isfinished", isFinished());
     */
  }

  public boolean isFinished() {
    return Math.abs(vars.avgDistTest * AUTO.measToPredictRatio) >= Math.abs(vars.totalDistance) - 0.1;
  }

  public void setEtherTurn(double angle) {
    vars.angle = angle;
  }

  public boolean isTurnFinished() {
    return Math.abs(vars.yaw) - Math.abs(vars.angle) <= 1;
  }

  /**
   * Check the TalonFX function for an error and print a message
   *
   * @param TalonFX
   * @param message  to print
   * @param printAll flag to print all (true) or just errors (false)
   * @return 1 for error and 0 for no error
   * @author
   */
  public int check(TalonFX motorController, String message, boolean printAll) {
    var rc = motorController.getLastError();
    if (rc != ErrorCode.OK || printAll) {
      System.out.println("[Talon] " + message + " " + rc);
    }
    return rc == ErrorCode.OK ? 0 : 1;
  }

  public void setMotionMagic(double dist, double angle) {
    startDrive();
    startTurn();
    vars.dist = dist;
    vars.avgDistInches = 0;
    vars.magicAngle = angle;
    vars.magicDistance = MathFormulas.inchesToNative(dist);

    topDriveLeft.configMotionAcceleration(MKDRIVE.maxNativeAcceleration);
    topDriveLeft.configMotionCruiseVelocity(MKDRIVE.maxNativeVelocity);

    topDriveRight.configMotionAcceleration(MKDRIVE.maxNativeAcceleration);
    topDriveRight.configMotionCruiseVelocity(MKDRIVE.maxNativeVelocity);

    bottomDriveLeft.configMotionAcceleration(MKDRIVE.maxNativeAcceleration);
    bottomDriveLeft.configMotionCruiseVelocity(MKDRIVE.maxNativeVelocity);

    bottomDriveRight.configMotionAcceleration(MKDRIVE.maxNativeAcceleration);
    bottomDriveRight.configMotionCruiseVelocity(MKDRIVE.maxNativeVelocity);
  }

  public void updateMotionMagic() {

    setModuleDrive(
        ControlMode.MotionMagic,
        vars.magicDistance,
        vars.magicDistance,
        vars.magicDistance,
        vars.magicDistance);
    setModuleTurn(vars.magicAngle, vars.magicAngle, vars.magicAngle, vars.magicAngle);
  }

  public boolean isMotionMagicDone() {
    double err = vars.dist - vars.avgDistInches;
    SmartDashboard.putNumber("err", err);
    return Math.abs(err) < 0.5 && Math.abs(vars.avgVelInches) < 0.1;
  }

  public double[] antiTip() {
    double setpoint = anti.calculate(-pigeon.getInstance().getPigPitch(), 0);
    // return new double[]{Math.cos(vars.yaw) * setpoint, Math.sin(vars.yaw) *
    // setpoint};
    return new double[] {
        -Math.cos(vars.yaw * (Math.PI / 180.0)) * setpoint,
        Math.sin(vars.yaw * (Math.PI / 180.0)) * setpoint
    };
  }

  private static class InstanceHolder {
    private static final MkSwerveTrain mInstance = new MkSwerveTrain();
  }

  public static class variables {

    // ether auto swerve values
    public double heading;
    public int side;
    public double distanceA;
    public double STRauto;
    public double FWDauto;
    public double RCWtemp;
    public double totalDistance;

    // motion magic values
    public double magicDistance;
    public double magicDistanceNative;
    public double magicAngle;

    // ether swerve values
    public double temp;
    public double yaw;
    public double A;
    public double B;
    public double C;
    public double D;
    public double[] mod1;
    public double[] mod2;
    public double[] mod3;
    public double[] mod4;
    public double max;

    // ether auto test values
    public double mod1Test;
    public double mod2Test;
    public double mod3Test;
    public double mod4Test;
    public double avgDistTest;
    public double maxTest;
    public double yawTest;
    public double tempTest;
    public double ATest;
    public double BTest;
    public double CTest;
    public double DTest;

    public double dist;

    // ether auto turn values
    public double angle;

    /** delta time */
    public double dt;

    /** Distance variable for driving in autonomous */
    public double straightDistance;

    /** Position of the driving motor in native units */
    public double posNativeTL, posNativeTR, posNativeBL, posNativeBR;

    /** Position of the driving motor in inches */
    public double posInchTL, posInchTR, posInchBL, posInchBR;

    /** Position of the driving motor in meters */
    public double posMetersTL, posMetersTR, posMetersBL, posMetersBR;

    /** Velocity of the driving motor in inches */
    public double velInchTL, velInchTR, velInchBL, velInchBR;

    /** Velocity of the driving motor in native units */
    public double velNativeTL, velNativeTR, velNativeBL, velNativeBR;

    /** Velocity of the driving motor in meters */
    public double velMetersTL, velMetersTR, velMetersBL, velMetersBR;

    /** Position of the turning motor in degrees */
    public double degTL, degTR, degBL, degBR;

    /** Driving motor values for autonomous */
    // public double[] output;

    /** Average velocity of driving motors in inches */
    public double avgVelInches;

    /** Average velocity of driving motors in native units */
    public double avgVelNative;

    /** Average distance of driving motors in inches */
    public double avgDistInches;

    public double avgDeg;

    public variables var;

    public double hP = 0.035, hI = 0.000, hD = 0.0024; // 0.03i, 0.01d
    // div rcw by 3 public double hP = 0.035, hI = 0.000, hD = 0.0024; // 0.03i,
    // 0.01d
    // TODO tune these so you dont need mkbaby for them to work
    // 0.015
    public double hIntegral, hDerivative, hPreviousError, hError;
    // code
    public double autoDist;

    public double autoDirectionTL;
    public double autoDirectionTR;
    public double autoDirectionBL;
    public double autoDirectionBR;

    public double[] pointOne;
    public double[] pointTwo;
  }
}
