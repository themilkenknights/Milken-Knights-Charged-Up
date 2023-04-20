// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AUTO.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SupaStruct;
import frc.robot.SupaStruct.Side;

public class ResetIntakeCommand extends CommandBase {
  /** Creates a new ResetIntake. */
  private SupaStruct supaKoopa = SupaStruct.getInstance();
  public ResetIntakeCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    supaKoopa.setResetDoneDiddlyDone(false, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    supaKoopa.zeroIntake(Side.Bottom);
    supaKoopa.zeroIntake(Side.Top);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return supaKoopa.getResetDoneDiddlyDoneBOTTOM() && supaKoopa.getResetDoneDiddlyDoneTOP();
  }
}
