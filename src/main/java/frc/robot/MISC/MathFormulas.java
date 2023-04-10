// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MISC;

import edu.wpi.first.math.util.Units;
import frc.robot.MISC.Constants.MKDRIVE;
import frc.robot.MISC.Constants.MKFALCON;

/** all MK math formulas */
public class MathFormulas {
  /*

                                                       E
                                                   ~~~~~~~~~
                                                ~~     +     ~~
                               /o----o\       ~~       + B     ~~       /o----o\
                               |  (F) |  (2) ~~        +        ~~ (1)  |  (F) |
                               \o----o/      ==========A==========      \o----o/
                                             \         |         /
                                              \        |        /
                                               \       |       /
                                                \      |D     /
                                                 \     |     /
                                                  \  __|__  /
                                                   \/  C  \/
                                                    \  |  /
                                                     \ | /
                                                      \|/

                               A = distanceA =
                               B = lengthB +
                               C = angle
                               D = radius |
                               E = circumference ~
                               F = robot
                               1 = starting position
                               2 = ending position
                               (diagram above isnt a hot air balloon fyi)
  */

  /**
   * Calculates a curved autonomous path's radius by using the distance between the starting and
   * ending point and the distance between the middle of the path and the height of the angular path
   *
   * @param distanceA
   * @param lengthB
   * @return Radius of the path
   */
  public static double calculateCircleRadius(double distanceA, double lengthB) {
    // return ((Math.pow(distanceA, 2) / 4) + Math.pow(lengthB, 2)) * (1 / (2 * lengthB));
    return ((lengthB / 2) + ((Math.pow(distanceA, 2)) / (8 * lengthB)));
  }

  /**
   * Calculates a curved autonomous path's circumference/length by using the distance between the
   * starting and ending point and the distance between the middle of the linear path and the max
   * height of the angular path
   *
   * @param distanceA
   * @param lengthB
   * @return Circumference of the linear path / distance of curved path
   * @see {@link #calculateCircleRadius(distanceA, lengthB)}
   */
  public static double calculateArcOfPath(double distanceA, double lengthB) {
    double radius = calculateCircleRadius(distanceA, lengthB);
    double theta =
        calculateAngleOfPath(
            distanceA, lengthB); // 2 * (Math.toDegrees((Math.asin((distanceA / (2 * radius))))));
    // return (theta / 360) * (2 * (Constants.kPi * radius));
    return Math.toRadians(theta * radius);
  }

  /**
   * Calculates a curved autonomous path's angle by using the distance between the starting and
   * ending point and the distance between the middle of the path and the height of the angular path
   *
   * @param distanceA
   * @param lengthB
   * @return Angle of the path (how much the angular motors have to turn in order to acheive this
   *     path)
   * @see {@link #calculateCircleRadius(distanceA, lengthB)}
   */
  public static double calculateAngleOfPath(double distanceA, double lengthB) {
    double radius = calculateCircleRadius(distanceA, lengthB);
    return 2 * (Math.toDegrees((Math.asin((distanceA / (2 * radius))))));
  }

  public static double nativeToInches(double nativeUnits) {
    return (nativeUnits / (MKFALCON.oneEncoderRotation * MKDRIVE.greerRatio))
        * MKDRIVE.wheelCircumference;
  }

  public static double inchesToNative(double in) {
    return (in / MKDRIVE.wheelCircumference) * (MKFALCON.oneEncoderRotation * MKDRIVE.greerRatio);
  }

  public static double nativePer100MstoInchesPerSec(double vel) {
    return 10 * nativeToInches(vel);
  }

  public static double inchesPerSecToUnitsPer100Ms(double vel) {
    return inchesToNative(vel) / 10;
  }

  public static double inchesToMeters(double inches) {
    return Units.inchesToMeters(inches);
  }

  public static double nativeToMeters(double nativeUnits) {
    return inchesToMeters(nativeToInches(nativeUnits));
  }

  public static double nativePer100MsToMetersPerSec(double nativeUnits) {
    return inchesToMeters(nativePer100MstoInchesPerSec(nativeUnits));
  }

  public static double metersToInches(double meters) {
    return Units.metersToInches(meters);
  }

  public static double metersPerSecondToNativeUnitsPer100Ms(double meters) {
    return inchesPerSecToUnitsPer100Ms(metersToInches(meters));
  }

  public static double nativeToDegrees(double gimmeRots, double greerRatio) {
    return (gimmeRots * 360) / (greerRatio * MKFALCON.oneEncoderRotation);
  }

  public static double nativePer100MsToMilesPerHour(double gimmeRots) {
    return nativePer100MstoInchesPerSec(gimmeRots) * .05681818181818181818181818;
  }

  public static double degreesToNative(double gimmeDeg, double greerRatio) {
    return (gimmeDeg * MKFALCON.oneEncoderRotation * greerRatio) / 360;
  }

  public static double nativePer100MsToInches(double gimmeNative, double dt) {
    return nativePer100MstoInchesPerSec(gimmeNative) / (1000 / dt);
  }

  /**
   * "Get the closest angle between the given angles."
   *
   * @param a angle a
   * @param b angle b
   * @return angle closest between the two angles
   * @author team 6624
   */
  public static double closestAngle(double a, double b) {
    double dir = (b % 360.0) - (a % 360.0);

    // convert from -360 to 360 to -180 to 180
    if (Math.abs(dir) > 180.0) {
      dir = -(Math.signum(dir) * 360.0) + dir;
    }
    return dir;
  }

  /**
   * "Get the closest angle between the given angles."
   *
   * @param a angle a
   * @param b angle b
   * @return angle closest between the two angles
   * @author team 6624
   */
  public static double closestAngleAuto(double a, double b) {
    double dir = (b % 360.0) - (a % 360.0);

    // convert from -360 to 360 to -180 to 180
    if (Math.abs(dir) > 180.0) {
      dir = -(Math.signum(dir) * 360.0) + dir;
    }
    return dir;
  }

  public static double setAutoDirection(double current, double setpoint) {
    // use the fastest way
    return current + closestAngle(current, setpoint);
  }

  public static double signumV4(double a) {
    if (a < 0) {
      return 180;
    } else {
      return 0;
    }
  }

  public static double signumAngleEdition(double a, double b) {
    if (a < 0 && b == 0) {
      return 360;
    } else {
      return 0;
    }
  }

  public static double[] optimize(double currentAngle, double[] mod) {
    double targetAngle = placeInAppropriate0To360Scope(currentAngle, mod[1]);
    double delta = targetAngle - currentAngle;
    if (Math.abs(delta) > 90) {
      targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
      mod[0] = Math.abs(mod[0]) * -1;
    }
    return new double[] {mod[0], targetAngle};
  }

  /**
   * @param scopeReference Current Angle
   * @param newAngle Target Angle
   * @return Closest angle within scope
   */
  public static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
    double lowerBound;
    double upperBound;
    double lowerOffset = scopeReference % 360;
    if (lowerOffset >= 0) {
      lowerBound = scopeReference - lowerOffset;
      upperBound = scopeReference + (360 - lowerOffset);
    } else {
      upperBound = scopeReference - lowerOffset;
      lowerBound = scopeReference - (360 + lowerOffset);
    }
    while (newAngle < lowerBound) {
      newAngle += 360;
    }
    while (newAngle > upperBound) {
      newAngle -= 360;
    }
    if (newAngle - scopeReference > 180) {
      newAngle -= 360;
    } else if (newAngle - scopeReference < -180) {
      newAngle += 360;
    }
    return newAngle;
  }

  public static double limit(double value, double min, double max) {
    if (value > max) {
      return max;
    } else if (value < min) {
      return min;
    } else {
      return value;
    }
  }

  public static double negativeLimit(double value, double min, double max) {
    if (Math.abs(value) > Math.abs(max)) {
      return max;
    } else if (Math.abs(value) < Math.abs(min)) {
      return min;
    } else {
      return value;
    }
  }

  public static double deadband(double val, double deadband) {
    return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
  }

  public static double limitAbsolute(double a, double max) {
    return Math.abs(a) < max ? a : Math.copySign(max, a);
  }

  public static double finalAngleRCW(double[] pointOne, double[] pointTwo) {
    return 90 - (Math.atan((pointTwo[1] - pointOne[1]) / (pointTwo[0] - pointOne[0])));
  }

  public static double calcA(double[] pointOne, double[] pointTwo) {
    return Math.sqrt(
        Math.pow(pointTwo[0] - pointOne[0], 2) + Math.pow(pointTwo[1] - pointOne[1], 2));
  }
}
