// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.util.UncleanStatusException;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.CANID;
import java.io.IOException;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  private Command m_autonomousCommand;

  PneumaticHub m_ph = new PneumaticHub(CANID.revphCANID);
  private MkSwerveTrain train = MkSwerveTrain.getInstance();
  private SupaStruct supaKoopa = SupaStruct.getInstance();
  private Timer timer;

  private PhotonCameraWrapper austin;
  private final Field2d mField2d = new Field2d();

  @Override
  public void robotInit() {

    SmartDashboard.setDefaultBoolean("Enable Compressor Analog", false);
    SmartDashboard.setDefaultBoolean("Disable Compressor", false);

    // Add number inputs for minimum and maximum pressure
    // SmartDashboard.setDefaultNumber("Minimum Pressure (PSI)", 100.0);
    // SmartDashboard.setDefaultNumber("Maximum Pressure (PSI)", 120.0);
    System.out.println("Robot enabled");
    // Starts recording to data log
    DataLogManager.start();
    try {

    } catch (UncleanStatusException ex) {
      DriverStation.reportError("Error creating Solenoid", ex.getStackTrace());
    }
    timer = new Timer();
    timer.start();

    train.startTrain();
    navx.getInstance().reset();
    try {
      austin = new PhotonCameraWrapper();
      SmartDashboard.putData(mField2d);
    } catch (IOException e) {

      e.printStackTrace();
    }
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Pressure", m_ph.getPressure(0));
    SmartDashboard.putBoolean("Compressor Running", m_ph.getCompressor());
    if (SmartDashboard.getBoolean("Enable Compressor Analog", false)) {
      SmartDashboard.putBoolean("Enable Compressor Analog", false);
      double minPressure = SmartDashboard.getNumber("Minimum Pressure (PSI)", 0.0);
      double maxPressure = SmartDashboard.getNumber("Maximum Pressure (PSI)", 120.0);
      m_ph.enableCompressorAnalog(minPressure, maxPressure);
    }
    if (SmartDashboard.getBoolean("Disable Compressor", false)) {
      SmartDashboard.putBoolean("Disable Compressor", false);
      m_ph.disableCompressor();
      CommandScheduler.getInstance().run();
    }
    if (austin != null) {
      Optional<EstimatedRobotPose> swerdlow = austin.photonPoseEstimator.update();
      if (swerdlow.isPresent()) {
        SmartDashboard.putString("Jared", swerdlow.get().estimatedPose.toString());
        mField2d.setRobotPose(swerdlow.get().estimatedPose.toPose2d());
      } else {
        SmartDashboard.putString("Jared", "nada");
      }
    }
  }

  @Override
  public void autonomousInit() {
    train.vars.avgDistTest = 0;
    m_autonomousCommand = null;
    train.startTrain();
    navx.getInstance().reset();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    train.updateSwerve();
  }

  @Override
  public void teleopInit() {
    supaKoopa.initTele();

    System.out.println("Robot Teleop Init");
    SmartDashboard.putNumber("distance total", Constants.AUTO.DISTANGLE.distance);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    train.startTrain();
    navx.getInstance().reset();
    //Arm.getInstance().setTelescope(MKTELE.minNativePositionTelescope);
    // Arm.getInstance().setLeft(MKARM.minNativePositionTelescope);
    // Arm.getInstance().setRight(MKARM.minNativePositionTelescope);
  }

  @Override
  public void teleopPeriodic() {
    supaKoopa.updateTele();
  }

  @Override
  public void disabledInit() {
    System.out.println("Robot disabled");
    supaKoopa.teleopDisabled();
    m_autonomousCommand = null;
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
    supaKoopa.initTest();
  }

  @Override
  public void testPeriodic() {
    train.updateSwerve();
    supaKoopa.updateTest();
  }
}
