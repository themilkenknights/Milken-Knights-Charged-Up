// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.MKAPRIL;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

/**Not JuneTags. Not FebruaryTags.*/
public class AprilTags {

    private PIDController turnPID;
    // Constants such as camera and target height stored. Change per robot and goal!
    private final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
    private final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);

    // Angle between horizontal and the camera.
    private final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

    // How far from the target we want to be
    private final double GOAL_RANGE_METERS = Units.feetToMeters(3);

    private PhotonCamera camera;

    private PhotonPipelineResult result;

    private AprilTags()
    {
        PortForwarder.add(5800, "photonvision.local", 5800);
        camera = new PhotonCamera("ShoutOutToMyStove");
        turnPID = new PIDController(MKAPRIL.kP, MKAPRIL.kI, MKAPRIL.kD);
        camera.setPipelineIndex(0);
        // Set driver mode to off.
        camera.setDriverMode(false);
    }

    public static AprilTags getInstance()
    {
        return InstanceHolder.mInstance;
    }

    /**Gets yaw, calculates pid for RCW for etherSwerve*/
    public double getRCWApril()
    {
        if (result.hasTargets()) {
            // First calculate range
            return turnPID.calculate(result.getBestTarget().getYaw(), 0); 
        }
        else
        {
            return 0.69;
        }
    }

    public void updateApril()
    {
        result = camera.getLatestResult();
    }

    /**Gets distance from tag*/
    public double getRange()
    {
        if (result.hasTargets()) {
        return
                    PhotonUtils.calculateDistanceToTargetMeters(
                            CAMERA_HEIGHT_METERS,
                            TARGET_HEIGHT_METERS,
                            CAMERA_PITCH_RADIANS,
                            Units.degreesToRadians(result.getBestTarget().getPitch()));
        }
        else 
        {
            return 0.69;
        }
    }

    public void aprilSmartDashboard()
    {
        SmartDashboard.putNumber("apirl", getRCWApril());
        SmartDashboard.putNumber("aprilRQANge", getRange());
        SmartDashboard.putBoolean("DOYOUFUCKIGSEEEE", result.hasTargets());
    }

    private static class InstanceHolder
    {
        private static final AprilTags mInstance = new AprilTags();
    } 
}
