// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AUTO.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.MECHANISMS.Intake;

public class IntakeAuto extends CommandBase {
  private double setpoint;

  /** Creates a new IntakeAuto. */
  public IntakeAuto(double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.setpoint = setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Intake.getInstance().rollerSet(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Intake.getInstance().rollerSet(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
