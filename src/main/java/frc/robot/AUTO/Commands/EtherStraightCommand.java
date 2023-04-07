// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AUTO.Commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.MECHANISMS.MkSwerveTrain;
import frc.robot.MISC.MathFormulas;

public class EtherStraightCommand extends CommandBase {
  private double dist, FWD, STR, angle;
  private MkSwerveTrain train = MkSwerveTrain.getInstance();

  public EtherStraightCommand(double dist, double FWD, double STR, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.dist = dist;
    this.FWD = FWD;
    this.STR = STR;
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    train.startDrive();
    train.setEtherAuto(dist, 0, 0);
    //System.out.println("are you finished in init: " + train.isFinished());
    //System.out.println("Test dist: " +  MkSwerveTrain.getInstance().vars.avgDistTest);
    //System.out.println("total dist: " + MkSwerveTrain.getInstance().vars.totalDistance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //SmartDashboard.putBoolean("FINISHED RAMP", train.isFinished());
    SmartDashboard.putNumber("testdist", MkSwerveTrain.getInstance().vars.avgDistTest);
    SmartDashboard.putNumber("totaldist", MkSwerveTrain.getInstance().vars.totalDistance);
    train.etherAutoSwerve(FWD, STR, MathFormulas.limit(train.moveToAngy(angle),-0.3,0.3), ControlMode.PercentOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //System.out.println(train.isFinished());
    //MkSwerveTrain.getInstance().etherSwerve(0, 0, 0, ControlMode.PercentOutput);
    train.setModuleDrive(ControlMode.PercentOutput, 0, 0, 0, 0);
    //train.setModuleTurn(0, 0, 0, 0);
    //train.setEtherAuto(dist, 0, 0);
    //train.resetRCWFinder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //System.out.println("mod2test" + MkSwerveTrain.getInstance().vars.mod2Test);
    //System.out.println("atest" + MkSwerveTrain.getInstance().vars.ATest);
    //System.out.println("Test dist isfinished: " +  MkSwerveTrain.getInstance().vars.avgDistTest);
    //System.out.println("total dist isfinished: " + MkSwerveTrain.getInstance().vars.totalDistance);
    return train.isFinished();
  }
}
