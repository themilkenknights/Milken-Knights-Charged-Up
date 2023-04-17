// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AUTO.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.MECHANISMS.Intake;

public class TopIntakeCOMMAND extends CommandBase {
  /** Creates a new IntakeCommand. */
  private double intakeState;

  private double rollerSpeed;
  private Intake intake = Intake.getInstance();

  public TopIntakeCOMMAND(double rollerSpeed, double intakeState) {
    this.rollerSpeed = rollerSpeed;
    this.intakeState = intakeState;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.moveTopIntakePID(intakeState);
    intake.movetoprollers(rollerSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.moveTopIntakePID(1);
    intake.movetoprollers(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
