// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.MKTRAIN;

/** Add your docs here. */
public class Odometry {
    Pose2d m_pose;

    ChassisSpeeds speeds = new ChassisSpeeds(3.0, -2.0, Math.PI);
    // Locations for the swerve drive modules relative to the robot center.
    Translation2d m_frontLeftLocation = new Translation2d(MKTRAIN.R, MKTRAIN.R);
    Translation2d m_frontRightLocation = new Translation2d(MKTRAIN.R, -MKTRAIN.R);
    Translation2d m_backLeftLocation = new Translation2d(-MKTRAIN.R, MKTRAIN.R);
    Translation2d m_backRightLocation = new Translation2d(-MKTRAIN.R, -MKTRAIN.R);

    // Creating my kinematics object using the module locations
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    // Convert to module states
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

    // Front left module state
    SwerveModuleState frontLeft = moduleStates[0];

    // Front right module state
    SwerveModuleState frontRight = moduleStates[1];

    // Back left module state
    SwerveModuleState backLeft = moduleStates[2];

    // Back right module state
    SwerveModuleState backRight = moduleStates[3];

    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
            m_kinematics, Rotation2d.fromDegrees(navx.getInstance().getNavxYaw()),
            new SwerveModulePosition[] {
                MkSwerveTrain.getInstance().modulePosTL(),
                MkSwerveTrain.getInstance().modulePosTR(),
                MkSwerveTrain.getInstance().modulePosBL(),
                MkSwerveTrain.getInstance().modulePosBR()
            }, new Pose2d(5.0, 13.5, new Rotation2d()));

    private Odometry() {

    }

    public static Odometry getInstance() {
        return InstanceHolder.mInstance;
      }

    public void updateOdometry() {
        // Update the pose
        m_pose = m_odometry.update(Rotation2d.fromDegrees(navx.getInstance().getNavxYaw()),
                new SwerveModulePosition[] {
                    MkSwerveTrain.getInstance().modulePosTL(),
                    MkSwerveTrain.getInstance().modulePosTR(),
                    MkSwerveTrain.getInstance().modulePosBL(),
                    MkSwerveTrain.getInstance().modulePosBR()
                });

    }

    public double getX()
    {
        return m_pose.getX();
    }

    public double getY()
    {
        return m_pose.getY();
    }

    public void updateSmartDashboard()
    {
        SmartDashboard.putNumber("x", MathFormulas.metersToInches(getX()));
        SmartDashboard.putNumber("y", MathFormulas.metersToInches(getY()));
    }

    private static class InstanceHolder {
        private static final Odometry mInstance = new Odometry();
      }
}
