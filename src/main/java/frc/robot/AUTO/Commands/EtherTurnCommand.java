// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AUTO.Commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.MECHANISMS.MkSwerveTrain;

public class EtherTurnCommand extends CommandBase {
  private double angle;
  private MkSwerveTrain train = MkSwerveTrain.getInstance();

  public EtherTurnCommand(double angle) {
    this.angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    train.setEtherTurn(angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    train.etherSwerve(0, 0, train.moveToAngy(angle) / 5, ControlMode.PercentOutput);
    //SmartDashboard.putNumber("move to angy", train.moveToAngy(angle));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    train.startDrive();
    train.vars.avgDistTest = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
