// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AUTO.Commandments;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AUTO.Commands.EtherStraightCommand;
import frc.robot.AUTO.Commands.ResetIntakeCommand;
import frc.robot.AUTO.Commands.TopIntakeCOMMAND;
import frc.robot.AUTO.Commands.BottomIntakeCOMMAND;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LeftSideAuto extends SequentialCommandGroup {
  /** Creates a new LeftSideAuto. */
  public LeftSideAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        Commands.deadline(new BottomIntakeCOMMAND(-1, 0).withTimeout(1)),
        Commands.deadline(new ResetIntakeCommand()),
        Commands.deadline(new EtherStraightCommand(195, -0.7, 0, 0), new TopIntakeCOMMAND(1, -36000).withTimeout(6)),
        Commands.deadline(new TopIntakeCOMMAND(1, -36000).withTimeout(1)),
        Commands.deadline(new EtherStraightCommand(195, 0.6, 0, 0), new TopIntakeCOMMAND(0, 0).withTimeout(5)),
        Commands.deadline(new BottomIntakeCOMMAND(-1, 0).withTimeout(2), new TopIntakeCOMMAND(1, 0).withTimeout(2)),
        Commands.deadline(new EtherStraightCommand(3, -0.6, -.1, 0)));
  }
}
