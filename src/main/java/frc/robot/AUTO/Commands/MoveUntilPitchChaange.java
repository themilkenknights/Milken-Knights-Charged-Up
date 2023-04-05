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
  private double thresh;
  private double speed;
  private double angle;
  private Condition cond;
  private boolean condition;
  private boolean isfinished;

  public MoveUntilPitchChaange(double thresh, double angle, double speed, Condition cond) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angle = angle;
    this.speed = speed;
    this.thresh = thresh;
    this.cond = cond;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (cond) {
      case LESSTHAN:
        condition = Math.abs(pitch) <= thresh;
        isfinished = Math.abs(pitch) > thresh;
        break;
      case GREATERTHAN:
        condition = Math.abs(pitch) >= thresh;
        isfinished = Math.abs(pitch) < thresh;
    }
    pitch = pigeon.getInstance().getPigPitch();
    SmartDashboard.putNumber("pitch", pitch);
    if (condition) {
      MkSwerveTrain.getInstance().etherSwerve(speed, 0, MkSwerveTrain.getInstance().moveToAngy(angle),
          ControlMode.PercentOutput);
    }
    SmartDashboard.putBoolean("isramp auto done", isfinished);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isfinished;
  }

  public enum Condition {
    GREATERTHAN,
    LESSTHAN
  };
}
