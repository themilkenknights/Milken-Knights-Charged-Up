// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MISC;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/** All variables that remain constant stored here */
public final class Constants {

  public static final double kPi = 3.14159265359;
  public static final double[] nullPID = {0, 0, 0, 0};

  // --------------------------------------------------------------------//
  // DRIVETRAIN
  // --------------------------------------------------------------------//
  public static class MKFALCON {
    public static final int velocityMeasAmount = 26;
    public static final int statusOneMeas = 20;
    public static final int statusTwoMeas = 20;
    public static final double voltComp = 11;
    public static final double oneEncoderRotation = 2048;
  }

  public static class MKDRIVE {
    public static final double kS = 0.1;
    public static final double kA = 0.1;
    public static final double kV = 0.1;

    public static final double maxNativeVelocity = 18000;
    public static final double maxNativeAcceleration = maxNativeVelocity / 4;

    public static final double kP = 0.05;
    public static final double kI = 0;
    public static final double kD = 0 * kP;
    public static final double kF = 0;

    public static final double[] pidf = {kP, kI, kD, kF};

    public static final NeutralMode mode = NeutralMode.Brake;

    public static final boolean inverted = false;

    public static final int scurve = 6;

    public static final double greerRatio = 6.75;

    public static final double wheelDiameterInches = 3.5;
    public static final double wheelCircumference = wheelDiameterInches * kPi;
  }

  public static class MKTURN {
    public static final double kP = 0.3;
    public static final double kI = 0; // 0.0003;
    // TODO test i in motion magic and pid for both auto and teleop, see if it
    // really is this
    public static final double kD = 0.0000000;
    public static final double kF = 0;

    public static final double[] pidf = {kP, kI, kD, kF};

    // TODO jack u want this?
    public static final NeutralMode mode = NeutralMode.Brake;

    public static final boolean inverted = true;

    public static final int scurve = 6;

    public static final double greerRatio = 150.0 / 7.0;
  }

  public static class MKCANCODER {
    public static final double topLeftOffset = -348.837890625; // 11.865234375 + 180;
    public static final double topRightOffset = -24.345703125; // -25.83984275 + 180;
    public static final double bottomLeftOffset =
        -128.14453125 + 180; // -209.091796875;// 153.0175781 - 16 - 180;
    public static final double bottomRightOffset = -353.05664062; // 7.294921875 - 180;

    public static final double[] offset = {
      MKCANCODER.topLeftOffset,
      MKCANCODER.topRightOffset,
      MKCANCODER.bottomLeftOffset,
      MKCANCODER.bottomRightOffset
    };

    public static final AbsoluteSensorRange range = AbsoluteSensorRange.Unsigned_0_to_360;

    public static final boolean inverted = false;
  }

  public static class MKTRAIN {
    public static final double L = 26.75;
    public static final double W = 18.75;

    public static final double widthInches = 32; // 28 was the old drive train
    public static final double heightInches = 24; // 28 was the old drive train

    public static final double R = Math.sqrt(Math.pow(L, 2) + Math.pow(W, 2));

    public static final double kP = 2;
    public static final double kI = 0;
    public static final double kD = 0;
  }

  public static class PIGEON {
    public static final double offsetYaw = 180;
    public static final double pitchThreshold = 2;
    public static final double rollThreashold = 20;
  }

  public static class MKBABY {
    public static final double fwdBABY = 1;
    public static final double strBABY = 1;
    public static final double rcwBABY = 3; // 2 very fast
  }

  // --------------------------------------------------------------------//
  // APRIL TAG
  // --------------------------------------------------------------------//
  public static class MKAPRIL {
    public static final double xkP = 0.05;
    public static final double xkI = 0;
    public static final double xkD = 0;

    public static final double ykP = 0.05;
    public static final double ykI = 0;
    public static final double ykD = 0;
    public static final Transform3d robotToCam =
        new Transform3d(
            new Translation3d(Units.inchesToMeters(-8), 0, Units.inchesToMeters(32)),
            new Rotation3d(
                0, 0,
                0)); // Cam mounted facing forward, half a meter forward of center, half a meter up
    // from center.
    public static final String cameraName = "ShoutOutToMyStove";
  }

  // --------------------------------------------------------------------//
  // AUTO RAMP
  // --------------------------------------------------------------------//
  public static class MKRAMP {
    public static final double kP = .007;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double threshold = 4;
  }

  // --------------------------------------------------------------------//
  // CONTROLLER PORTS
  // --------------------------------------------------------------------//
  public static class CONTROLLERS {
    public static final int driverPort = 0;
    public static final int opPort = 1;

    public static class DriveInput {
      public static final int fwd = 1;
      public static final int str = 0;
      public static final int rcwX = 4;
      public static final int rcwY = 5;
      // 1 a, 2 b, 3 x, 4 y
    }

    public static final int topPOV = 0;
    public static final int rightPOV = 90;
    public static final int bottomPOV = 180;
    public static final int leftPOV = 270;
  }

  // --------------------------------------------------------------------//
  // CANIDS
  // --------------------------------------------------------------------//
  public static class CANID {
    // drive motors
    public static final int topDriveLeftCANID = 5; // 5
    public static final int topDriveRightCANID = 7;
    public static final int bottomDriveLeftCANID = 9;
    public static final int bottomDriveRightCANID = 3; // 3

    // turn motors
    public static final int topTurnLeftCANID = 6; // 6
    public static final int topTurnRightCANID = 8;
    public static final int bottomTurnLeftCANID = 1;
    public static final int bottomTurnRightCANID = 4; // 4

    // cancoder
    public static final int topTurnLeftCANCoderCANID = 18; // 18
    public static final int topTurnRightCANCoderCANID = 17;
    public static final int bottomTurnLeftCANCoderCANID = 15; // BAD
    public static final int bottomTurnRightCANCoderCANID = 16; // 16

    // intakes
    public static final int topLeftIntakeCANID = 57;
    public static final int topRightIntakeCANID = 62;
    public static final int bottomLeftIntakeCANID = 60;
    public static final int bottomRightIntakeCANID = 61;
    public static final int toprollersCANID = 59;
    public static final int bottomrollersCANID = 58;

    // revh ph

    public static final int revpdhCANID = 23; // MUST MAKE SURE IT IS ON RIO NOT CANIVORE

    public static final int pigeonCANID = 30;
  }

  // --------------------------------------------------------------------//
  // INTAKE
  // --------------------------------------------------------------------//
  public static class MKINTAKE {
    public static final NeutralMode rollerNeutralMode = NeutralMode.Coast;
    public static final NeutralMode intakeNeutralMode = NeutralMode.Brake;

    public static final double kP = 0.00002;
    public static final double kI = 0.0000001;
    public static final double kD = 0.0000;
    public static final double kF = 0;

    public static final double[] pidf = {kP, kI, kD, kF};

    public static final boolean topLeftInverted = true;
    public static final boolean topRightInverted = false;
    public static final boolean bottomLeftInverted = true;
    public static final boolean bottomRightInverted = false;

    public static final double rollerPercentSpeed = .7;
    public static final double intakePercentSpeed = .7;

    public static final double greerRatio = 20.0 / 18.0 / 45.0;

    public static final double topOutNative = 0;
    public static final double bottomOutNative = 0;
  }

  public static class AUTO {

    public static final double measToPredictRatio = 0.823270434926127; // 0.93320900560016;

    public static class DISTANGLE {
      public static final double distanceA = 80;
      public static final double lengthB = 30;

      public static final int sidePos = 1;
      public static final int sideCon = -1;

      public static final double headinguno = 90;
      public static final double headingdos = -90;
      public static final double headingtres = 90;
      public static final double headingquad = -90;

      public static final double headingsinco = 270;
      public static final double headingsix = -270;
      public static final double headingsev = 270;
      public static final double headingocto = -270;

      public static final double headingnine = 0;
      public static final double headingten = 0;
      public static final double headingele = 360;
      public static final double headingtwel = 180;

      public static final double headingthir = -360;
      public static final double headingfourt = -180;
      public static final double headingfif = -360;
      public static final double headingsixt = -180;

      public static final double distance = MathFormulas.calculateArcOfPath(distanceA, lengthB);
      public static final double angle = MathFormulas.calculateAngleOfPath(distanceA, lengthB);
    }

    // --------------------------------------------------------------------//
    // WPI SWERVE
    // --------------------------------------------------------------------//
    public static final double turnSwerveControlKp = 1;
    public static final double driveSwerveControlKpY = 1;
    public static final double driveSwerveControlKpX = 1;

    public static final double heightMeters = MathFormulas.inchesToMeters(MKTRAIN.heightInches / 2);
    public static final double widthMeters = MathFormulas.inchesToMeters(MKTRAIN.widthInches / 2);

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(heightMeters, widthMeters),
            new Translation2d(heightMeters, -widthMeters),
            new Translation2d(-heightMeters, widthMeters),
            new Translation2d(-heightMeters, -widthMeters));

    // actual drive module stats
    public static final double maxModuleTurnVelo = kPi;
    public static final double maxModuleTurnAccel = kPi;

    // actual drive module stats
    public static final double maxModuleDriveVelo = 1;
    public static final double maxModuleDriveAccel = 1;

    // for turning constraints
    public static final double maxAutoTurnVelo = kPi;
    public static final double maxAutoTurnAccel = kPi;

    // for trajectory config
    public static final double maxAutoDriveVelo = 1; // 2;
    public static final double maxAutoDriveAccel = 1; // 2;

    public static final double maxDriveVelo = 1;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(maxAutoTurnVelo, maxAutoTurnAccel);
  }

  // --------------------------------------------------------------------//
  // ODOEMTRY
  // --------------------------------------------------------------------//
  public static class ODO {
    public static final double goalXInches = 120;
    public static final double goalYInches = 120;
    public static final double goalRadius = 60;
  }

  // --------------------------------------------------------------------//
  // LIGHTS
  // --------------------------------------------------------------------//
  public static class LIGHTS {
    public static final int PWMPORT = 0;
    public static final int bufferNum = 151;
    public static final int MaxRGBValue = 60;
  }
}
