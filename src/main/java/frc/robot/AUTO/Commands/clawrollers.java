// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AUTO.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.MECHANISMS.ARM.Wrist;

public class clawrollers extends CommandBase {
  private double setpoint;
  /** Creates a new clawrollers. */
  public clawrollers(double setpoint) {
    this.setpoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Wrist.getInstance().moveWristRoller(setpoint);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Wrist.getInstance().moveWristRoller(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
