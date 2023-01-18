// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.util.UncleanStatusException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
  private MkSwerveTrain train = MkSwerveTrain.getInstance();
  private SupaStruct supaKoopa = SupaStruct.getInstance();
  private Timer timer;

  @Override
  public void robotInit() 
  {
        System.out.println("Robot enabled");
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
  public void robotPeriodic() 
  {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() 
  {
    train.vars.avgDistTest=0;
    m_autonomousCommand = new Ramp();
    train.startTrain();
    navx.getInstance().reset();
    if (m_autonomousCommand != null) 
    {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() 
  {
    train.updateSwerve();
  }

  @Override
  public void teleopInit() 
  {
    supaKoopa.initTele();
    
        System.out.println("Robot Teleop Init");
    SmartDashboard.putNumber("distance total", Constants.AUTO.DISTANGLE.distance);
    if (m_autonomousCommand != null) 
    {
      m_autonomousCommand.cancel();
    }
    train.startTrain();
    navx.getInstance().reset();
  }
  
  @Override
  public void teleopPeriodic() 
  {
    supaKoopa.updateTele();
  }

  @Override
  public void disabledInit()
  {
        System.out.println("Robot disabled");
    supaKoopa.teleopDisabled();
    m_autonomousCommand = new Ramp();
  }

  @Override
  public void disabledPeriodic()
  {

  }

  @Override
  public void testInit() 
  {
    supaKoopa.initTest();
  }

  @Override
  public void testPeriodic() 
  {
    train.updateSwerve();
    supaKoopa.updateTest();
  }

  
}
