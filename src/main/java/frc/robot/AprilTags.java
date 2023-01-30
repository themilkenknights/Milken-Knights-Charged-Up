// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.MKAPRIL;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

/** Not JuneTags. Not FebruaryTags. X is up, Y is side to side when facing directly at it */
public class AprilTags {

  private PIDController moveAprilX;
  private PIDController moveAprilZ;
  private PIDController moveAprilY;

  // Constants such as camera and target height stored. Change per robot and goal!
  private final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
  private final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);

  // Angle between horizontal and the camera.
  private final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

  // How far from the target we want to be
  private final double GOAL_RANGE_METERS = Units.feetToMeters(3);

  private PhotonCamera camera;

  private PhotonPipelineResult result;

  private MkSwerveTrain train = MkSwerveTrain.getInstance();

  private AprilTags() {
    PortForwarder.add(5800, "photonvision.local", 5800);

    camera = new PhotonCamera("ShoutOutToMyStove");

    moveAprilX = new PIDController(MKAPRIL.xkP, MKAPRIL.xkI, MKAPRIL.xkD);
    moveAprilZ = new PIDController(MKAPRIL.zkP, MKAPRIL.zkI, MKAPRIL.zkD);
    moveAprilY = new PIDController(MKAPRIL.ykP, MKAPRIL.ykI, MKAPRIL.ykD);

    camera.setPipelineIndex(0);

    // Set driver mode to off.
    camera.setDriverMode(false);
  }

  public static AprilTags getInstance() {
    return InstanceHolder.mInstance;
  }

  public void updateApril() {
    result = camera.getLatestResult();
  }

  /** Gets distance from tag */
  public double get2DRange() {
    if (result.hasTargets()) {
      return PhotonUtils.calculateDistanceToTargetMeters(
          CAMERA_HEIGHT_METERS,
          TARGET_HEIGHT_METERS,
          CAMERA_PITCH_RADIANS,
          Units.degreesToRadians(result.getBestTarget().getPitch()));
    } else {
      return 0.69;
    }
  }

  public double getAxis(String axis) {
    axis = axis.toLowerCase();
    if (result.hasTargets()) {
      if (axis == "x") {
        return result.getBestTarget().getBestCameraToTarget().getX();
      } else if (axis == "y") {
        return result.getBestTarget().getBestCameraToTarget().getY();
      } else if (axis == "z") {
        return result.getBestTarget().getBestCameraToTarget().getZ();
      } else if (axis == "r") {
        return (Math.toDegrees(
                    result.getBestTarget().getBestCameraToTarget().getRotation().getAngle())
                - 180)
            % 360;
      } else if (axis == "yaw") {
        return result.getBestTarget().getYaw();
      } else {
        return 0.69;
      }
    } else {
      return 0.69;
    }
  }

  public void alignToTag() {
    if (result.hasTargets()) {

      double xPID;
      double zPID;
      double yPID;

      xPID = moveAprilX.calculate(getAxis("x"), 1);
      zPID = moveAprilZ.calculate(getAxis("r"), 0);
      yPID = 0;

      if (result.getBestTarget().getPoseAmbiguity() > 0.2) {
        zPID = 0;
      }
      if (Math.abs(zPID) <= 0.1 && Math.abs(getAxis("r")) <= 3)
        ;
      {
        zPID = 0;
        moveAprilY.calculate(getAxis("y"), 0);
      }

      SmartDashboard.putNumber("xpid", xPID);

      train.etherSwerve(
          MathFormulas.limit(xPID, -0.5, 0.5),
          MathFormulas.limit(-yPID, -0.5, 0.5),
          MathFormulas.limit(-zPID, -0.3, 0.3),
          ControlMode.PercentOutput);
    } else {
      train.etherSwerve(0, 0, 0, ControlMode.PercentOutput);
    }
  }

  public void aprilSmartDashboard() {
    SmartDashboard.putNumber("aprilRQANge", get2DRange());
    SmartDashboard.putBoolean("DOYOUFUCKIGSEEEE", result.hasTargets());
    SmartDashboard.putNumber("z", getAxis("r"));
    SmartDashboard.putNumber("z2", getAxis("yaw"));
    SmartDashboard.putNumber("zpid", moveAprilZ.calculate(getAxis("r"), 0));
    SmartDashboard.putNumber("ypid", moveAprilY.calculate(getAxis("y"), 0));
  }

  private static class InstanceHolder {
    private static final AprilTags mInstance = new AprilTags();
  }
}
