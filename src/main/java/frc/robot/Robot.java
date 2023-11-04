// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.hal.util.UncleanStatusException;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.AUTO.Commandments.BumpSide;
import frc.robot.AUTO.Commandments.LeftSideAuto;
import frc.robot.AUTO.Commandments.RampAuto;
import frc.robot.AUTO.Commandments.TestResetIntake;
import frc.robot.MECHANISMS.MkSwerveTrain;
import frc.robot.MISC.SwerveDrivePoseEstimator;
import frc.robot.MISC.pigeon;

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

  private SendableChooser<AutoPosition> positionChooser = new SendableChooser<>();
  private ShuffleboardTab mTab = Shuffleboard.getTab("Match");
  private ComplexWidget positionChooserTab =
      mTab.add("Auto Chooser", positionChooser).withWidget(BuiltInWidgets.kSplitButtonChooser);
  private MkSwerveTrain train = MkSwerveTrain.getInstance();
  private SupaStruct supaKoopa = SupaStruct.getInstance();

  @Override
  public void robotInit() {
    positionChooser.setDefaultOption("DOUBLESIDE", AutoPosition.LEFTSIDEDOUBLE);
    positionChooser.addOption("MIDDLE", AutoPosition.MIDDLE);
    //positionChooser.addOption("BUMP", AutoPosition.BUMP);
    System.out.println("Robot enabled");
    train.startTrain();
    pigeon.getInstance().reset();
    CameraServer.startAutomaticCapture();
    }

    
  

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    //SwerveDrivePoseEstimator(kinematics,pigeon.getInstance(),)
  }

  @Override
  public void autonomousInit() {
    System.out.println("i am auito  initialzies");
    train.vars.avgDistTest = 0;
    switch (positionChooser.getSelected()) {
      case MIDDLE:
        m_autonomousCommand = new RampAuto();
        break;
      case LEFTSIDEDOUBLE:
        m_autonomousCommand = new LeftSideAuto();
        break;
     // case BUMP:
       // m_autonomousCommand = new BumpSide();
        //break;
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
  }

  @Override
  public void teleopInit() {
    //Shuffleboard.selectTab("SmartDashboard");

    supaKoopa.initTele();

    System.out.println("Robot Teleop Init");
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    train.startTrain();
    pigeon.getInstance().reset();
  }

  @Override
  public void teleopPeriodic() {
    supaKoopa.updateTele();
  }

  @Override
  public void disabledInit() {
    m_autonomousCommand = null;
    System.out.println("Robot disabled");
    supaKoopa.teleopDisabled();
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

  public enum AutoPosition {
    LEFTSIDEDOUBLE,
    RIGHTSIDEDOUBLE,
    RAMPAUTO,
    MIDDLE,
    BUMP
  }
}
