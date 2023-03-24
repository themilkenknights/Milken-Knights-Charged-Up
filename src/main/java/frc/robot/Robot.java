// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.hal.util.UncleanStatusException;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.AUTO.Commandments.autopaths.LeftDoubleLow;
import frc.robot.AUTO.Commandments.autopaths.RightDoubleLow;
import frc.robot.AUTO.Commandments.autopaths.lowerLinkRight;
import frc.robot.MECHANISMS.ARM.Arm;
import frc.robot.MECHANISMS.ARM.Wrist;
import frc.robot.MECHANISMS.MkSwerveTrain;
import frc.robot.MISC.Constants.CANID;
import frc.robot.MISC.Constants.MKTELE;
import frc.robot.MISC.Odometry;
import frc.robot.MISC.pigeon;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  private Command m_autonomousCommand;

  private SendableChooser<AutoPosition> positionChooser = new SendableChooser<>();
  private ShuffleboardTab mTab = Shuffleboard.getTab("Match");
  private ComplexWidget positionChooserTab = mTab.add("Auto Chooser", positionChooser)
      .withWidget(BuiltInWidgets.kSplitButtonChooser);
  PneumaticHub m_ph = new PneumaticHub(CANID.revphCANID);
  private MkSwerveTrain train = MkSwerveTrain.getInstance();
  private SupaStruct supaKoopa = SupaStruct.getInstance();
  private Timer timer;
  private int lightMode = 0;
  private UsbCamera usbCamera;
  private boolean resetDoneDiddlyDone = false;

  // Creates UsbCamera and MjpegServer [1] and connects them

  @Override
  public void robotInit() {
    resetDoneDiddlyDone = false;
    PortForwarder.add(5800, "photonvision.local", 5800);
    positionChooser.setDefaultOption("LEFTDOUBLE", AutoPosition.LEFTSIDEDOUBLE);
    Arm.getInstance().getTelescopeMotor().setNeutralMode(NeutralMode.Brake);
    CameraServer.startAutomaticCapture();
    Shuffleboard.selectTab("Match");
    positionChooser.addOption("RIGHTDOUBLE", AutoPosition.RIGHTSIDEDOUBLE);
    positionChooser.addOption("MIDDLE", AutoPosition.MIDDLE);
    positionChooser.addOption("LOWERLINKRIGHT", AutoPosition.RIGHTLOWERLINK);
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
    pigeon.getInstance().reset();
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putBoolean("whodoneit", resetDoneDiddlyDone);
    
    Odometry.getInstance().updateOdometry(supaKoopa.getAprilEnabled());
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
    Arm.getInstance().setTelescope(125);
    switch (positionChooser.getSelected()) {
      case RIGHTSIDEDOUBLE:
        m_autonomousCommand = new RightDoubleLow();
        break;
      case MIDDLE:
        m_autonomousCommand = new LeftDoubleLow();
        break;
      case RIGHTLOWERLINK:
        m_autonomousCommand = new lowerLinkRight();
        break;
    }

    train.startTrain();
    pigeon.getInstance().reset();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    train.updateSwerve();
    Arm.getInstance().updateSmartdashboard();
  }

  @Override
  public void teleopInit() {
    Shuffleboard.selectTab("SmartDashboard");
    supaKoopa.initTele();
    SmartDashboard.putBoolean("isreset", Wrist.getInstance().getLimitSwitch());

    System.out.println("Robot Teleop Init");
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    train.startTrain();
    pigeon.getInstance().reset();
    Arm.getInstance().setTelescope(MKTELE.minNativePositionTelescope);
    // Arm.getInstance().setLeft(MKARM.minNativePositionTelescope);
    // Arm.getInstance().setRight(MKARM.minNativePositionTelescope);
  }

  @Override
  public void teleopPeriodic() {
    if (!resetDoneDiddlyDone) {
      Wrist.getInstance().moveWristMotor(0.2);
      resetDoneDiddlyDone = Wrist.getInstance().getLimitSwitch();
      if(resetDoneDiddlyDone)
      {
        Wrist.getInstance().moveWristMotor(0);
      }
    }
    supaKoopa.updateTele();
  }

  @Override
  public void disabledInit() {
    System.out.println("Robot disabled");
    supaKoopa.teleopDisabled();
    m_autonomousCommand = new RightDoubleLow();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
    supaKoopa.initTest();
  }

  @Override
  public void testPeriodic() {
    train.updateSwerve();
    supaKoopa.updateTest();
  }

  public enum AutoPosition {
    LEFTSIDEDOUBLE,
    RIGHTSIDEDOUBLE,
    RIGHTLOWERLINK,
    MIDDLE
  }
}
