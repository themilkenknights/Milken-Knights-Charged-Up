// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AUTO.Commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.MECHANISMS.MkSwerveTrain;
import frc.robot.MISC.pigeon;

public class MoveUntilPitchChaange extends CommandBase {
  /** Creates a new MoveUntilPitchChaange. */
  private double pitch;
  private double angle;
  public MoveUntilPitchChaange(double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
  this.angle = angle;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pitch = pigeon.getInstance().getPigPitch();
    SmartDashboard.putNumber("pitch", pitch);
    if(Math.abs(pitch)<=4.2)
    {
      MkSwerveTrain.getInstance().etherSwerve(0.2, 0, MkSwerveTrain.getInstance().moveToAngy(angle), ControlMode.PercentOutput);
    }
    SmartDashboard.putBoolean("isramp auto done", Math.abs(pitch) > 4.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(pitch) > 4.2;
  }
}
