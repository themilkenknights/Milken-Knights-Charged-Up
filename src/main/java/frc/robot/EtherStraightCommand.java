// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;

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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    train.etherAutoSwerve(FWD, STR, train.moveToAngy(angle), ControlMode.PercentOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return train.isFinished();
  }
}
