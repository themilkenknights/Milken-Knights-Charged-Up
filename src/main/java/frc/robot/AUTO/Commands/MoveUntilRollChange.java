// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AUTO.Commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.MECHANISMS.MkSwerveTrain;
import frc.robot.MISC.pigeon;

public class MoveUntilRollChange extends CommandBase {
  /** Creates a new MoveUntilRollChange. */
  private double roll;

  private double thresh;
  private double speed;
  private double angle;
  private Condition cond;
  private boolean condition;
  private boolean isfinished;

  public MoveUntilRollChange(double thresh, double angle, double speed, Condition cond) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angle = angle;
    this.speed = speed;
    this.thresh = thresh;
    this.cond = cond;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    roll = pigeon.getInstance().getPigRoll();
    switch (cond) {
      case LESSTHAN:
        condition = roll <= thresh;
        isfinished = roll > thresh;
        break;
      case GREATERTHAN:
        condition = roll >= thresh;
        isfinished = roll < thresh;
    }
    SmartDashboard.putNumber("roll", roll);
    SmartDashboard.putBoolean("condition", condition);
    if (condition) {
      MkSwerveTrain.getInstance()
          .etherSwerve(
              speed,
              0,
              MkSwerveTrain.getInstance().moveToAngy(angle) / 3,
              ControlMode.PercentOutput); // MkSwerveTrain.getInstance().moveToAngy(angle),
    }
    SmartDashboard.putBoolean("isramp auto done: " + thresh, isfinished);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    MkSwerveTrain.getInstance().etherSwerve(0, 0, 0, ControlMode.PercentOutput);
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
