// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.hal.util.UncleanStatusException;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.AUTO.Commandments.AutoOne;
import frc.robot.MECHANISMS.MkSwerveTrain;
import frc.robot.MISC.Constants;
import frc.robot.MISC.Constants.CANID;
import frc.robot.MISC.Odometry;
import frc.robot.MISC.navx;

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
  private AutoOne m_autonomousCommand;

  PneumaticHub m_ph = new PneumaticHub(CANID.revphCANID);
  private MkSwerveTrain train = MkSwerveTrain.getInstance();
  private SupaStruct supaKoopa = SupaStruct.getInstance();
  private Timer timer;

  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();
    // SmartDashboard.setDefaultBoolean("Enable Compressor Analog", false);
    // SmartDashboard.setDefaultBoolean("Disable Compressor", false);

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
  }

  @Override
  public void robotPeriodic() {
    Odometry.getInstance().updateOdometry();
    CommandScheduler.getInstance().run();
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
  }

  @Override
  public void autonomousInit() {
    train.vars.avgDistTest = 0;
    m_autonomousCommand = new AutoOne();
    train.startTrain();
    navx.getInstance().reset();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    train.updateSwerve();
    SmartDashboard.putBoolean("ARE YOU FUCKING WORKING", m_autonomousCommand.isFinished());
    SmartDashboard.putBoolean("ARE YOU FUCKING SCHEDULED", m_autonomousCommand.isScheduled());
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
    // Arm.getInstance().setTelescope(MKTELE.minNativePositionTelescope);
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
    m_autonomousCommand = new AutoOne();
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
