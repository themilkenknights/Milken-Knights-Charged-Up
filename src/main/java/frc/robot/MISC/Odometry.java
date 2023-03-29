// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MISC;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CAMERA.PhotonCameraWrapper;
import frc.robot.MECHANISMS.MkSwerveTrain;
import frc.robot.MISC.Constants.MKTRAIN;
import java.io.IOException;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

/** Add your docs here. */
// TODO fix odo and april vision
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
      m_kinematics,
      Rotation2d.fromDegrees(pigeon.getInstance().getPigYaw()),
      new SwerveModulePosition[] {
          MkSwerveTrain.getInstance().modulePosTL(),
          MkSwerveTrain.getInstance().modulePosTR(),
          MkSwerveTrain.getInstance().modulePosBL(),
          MkSwerveTrain.getInstance().modulePosBR()
      },
      new Pose2d(5.0, 13.5, new Rotation2d()));

  SwerveDrivePoseEstimator m_SwerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
      m_kinematics,
      Rotation2d.fromDegrees(pigeon.getInstance().getPigYaw()),
      new SwerveModulePosition[] {
          MkSwerveTrain.getInstance().modulePosTL(),
          MkSwerveTrain.getInstance().modulePosTR(),
          MkSwerveTrain.getInstance().modulePosBL(),
          MkSwerveTrain.getInstance().modulePosBR()
      },
      new Pose2d(5.0, 13.5, new Rotation2d()));

  private PhotonCameraWrapper austin;
  private final Field2d mField2d = new Field2d();

  private Odometry() {
    try {
      austin = new PhotonCameraWrapper();
      SmartDashboard.putData(mField2d);
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  public static Odometry getInstance() {
    return InstanceHolder.mInstance;
  }

  public void updateOdometry(boolean enableApril) {
    // Update the pose
    m_SwerveDrivePoseEstimator.update(
        Rotation2d.fromDegrees(pigeon.getInstance().getPigYaw()),
        new SwerveModulePosition[] {
            MkSwerveTrain.getInstance().modulePosTR(),
            MkSwerveTrain.getInstance().modulePosBL(),
            MkSwerveTrain.getInstance().modulePosTL(),
            MkSwerveTrain.getInstance().modulePosBR()
        });
    if (enableApril) {
      Optional<EstimatedRobotPose> result = austin
          .getEstimatedGlobalPose(m_SwerveDrivePoseEstimator.getEstimatedPosition());
      if (result.isPresent()) {
        EstimatedRobotPose camPose = result.get();
        m_SwerveDrivePoseEstimator.addVisionMeasurement(
            camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
      }
    }
    mField2d.setRobotPose(m_SwerveDrivePoseEstimator.getEstimatedPosition());
  }

  public Pose2d getPose() {
    return m_pose;
  }

  public double getX() {
    return m_pose.getX();
  }

  public double getY() {
    return m_pose.getY();
  }

  public void resetToPose2D(double x, double y, double rot) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(rot),
        new SwerveModulePosition[] {
            MkSwerveTrain.getInstance().modulePosTL(),
            MkSwerveTrain.getInstance().modulePosTR(),
            MkSwerveTrain.getInstance().modulePosBL(),
            MkSwerveTrain.getInstance().modulePosBR()
        },
        new Pose2d(new Translation2d(x + getX(), y + getY()), Rotation2d.fromDegrees(rot)));
  }

  public void reset() {
    MkSwerveTrain.getInstance().resetDrive();
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(0),
        new SwerveModulePosition[] {
            MkSwerveTrain.getInstance().modulePosTL(),
            MkSwerveTrain.getInstance().modulePosTR(),
            MkSwerveTrain.getInstance().modulePosBL(),
            MkSwerveTrain.getInstance().modulePosBR()
        },
        new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
  }

  public void updateSmartDashboard() {
    // SmartDashboard.putNumber("xodo", MathFormulas.metersToInches(getX()));
    // SmartDashboard.putNumber("yodo", MathFormulas.metersToInches(getY()));
  }

  private static class InstanceHolder {
    private static final Odometry mInstance = new Odometry();
  }
}
