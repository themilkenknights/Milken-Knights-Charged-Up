// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.MKAPRIL;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix.motorcontrol.ControlMode;

/**Not JuneTags. Not FebruaryTags. X is up, Y is side to side when facing directly at it*/
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

    private AprilTags()
    {
        PortForwarder.add(5800, "photonvision.local", 5800);
        
        camera = new PhotonCamera("ShoutOutToMyStove");
        
        moveAprilX = new PIDController(MKAPRIL.xkP, MKAPRIL.xkI, MKAPRIL.xkD);
        moveAprilZ = new PIDController(MKAPRIL.zkP, MKAPRIL.zkI, MKAPRIL.zkD);
        moveAprilY = new PIDController(MKAPRIL.ykP, MKAPRIL.ykI, MKAPRIL.ykD);
        
        camera.setPipelineIndex(0);
        
        // Set driver mode to off.
        camera.setDriverMode(false);
    }

    public static AprilTags getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public void updateApril()
    {
        result = camera.getLatestResult();
    }

    /**Gets distance from tag*/
    public double get2DRange()
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

    public double getAxis(String axis)
    {
        axis = axis.toLowerCase();
        if (result.hasTargets()) {
            if(axis == "x")
            {
                return result.getBestTarget().getBestCameraToTarget().getX();
            }
            else if(axis == "y")
            {
                return result.getBestTarget().getBestCameraToTarget().getY();
            }
            else if(axis == "z")
            {
                return result.getBestTarget().getBestCameraToTarget().getZ();
            }
            else if(axis == "r")
            {
                return Math.toDegrees(result.getBestTarget().getBestCameraToTarget().getRotation().getAngle());
            }
            else 
            {
                return 0.69;
            }
        }
        else
        {
            return 0.69;
        }
    }

    public void alignToTag()
    {
        if (result.hasTargets())
        {
            double xPID = moveAprilX.calculate(getAxis("x"), 1);
            double zPID = moveAprilZ.calculate(getAxis("z"), 180);
            
            SmartDashboard.putNumber("xpid", xPID);
            SmartDashboard.putNumber("zpid", zPID);

            train.etherSwerve(MathFormulas.limit(xPID, -0.5, 0.5), 
                              MathFormulas.limit(0, -0.5, 0.5), 
                              MathFormulas.limit(0, -0.5, 0.5),
                              ControlMode.PercentOutput);
        }
        else 
        {
            train.etherSwerve(0, 0, 0, ControlMode.PercentOutput);
        }
    }


    public void aprilSmartDashboard()
    {
        SmartDashboard.putNumber("aprilRQANge", get2DRange());
        SmartDashboard.putBoolean("DOYOUFUCKIGSEEEE", result.hasTargets());
        SmartDashboard.putNumber("z", getAxis("z"));
    }

    private static class InstanceHolder
    {
        private static final AprilTags mInstance = new AprilTags();
    } 
}
