// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MotionMagicAuto extends CommandBase {
  private double dist;
  private double angle;
  /** Creates a new MotionMagicAuto. */
  public MotionMagicAuto(double dist, double angle) {
    this.angle = angle;
    this.dist = dist;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    MkSwerveTrain.getInstance().setMotionMagic(dist, angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    MkSwerveTrain.getInstance().updateMotionMagic();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("idone", MkSwerveTrain.getInstance().isMotionMagicDone());
    return MkSwerveTrain.getInstance().isMotionMagicDone();
  }
}
