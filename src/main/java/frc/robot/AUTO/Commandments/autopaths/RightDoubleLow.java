// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AUTO.Commandments.autopaths;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AUTO.Commands.EtherStraightCommand;
import frc.robot.AUTO.Commands.IntakeAuto;
import frc.robot.AUTO.Commands.intakedeploy;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RightDoubleLow extends SequentialCommandGroup {
  /** Creates a new RightDoubleLow. */
  public RightDoubleLow() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        Commands.deadline(new EtherStraightCommand(0, -0.5, 0, 0)),
        Commands.deadline(new IntakeAuto(.8).withTimeout(1)),
        Commands.deadline(new EtherStraightCommand(100, -0.5, 0, 0.3)),
        Commands.deadline(new EtherStraightCommand(79, -0.5, 0, 110)),
        Commands.deadline(new intakedeploy().withTimeout(1)),
        Commands.deadline( new EtherStraightCommand(60, -.1, -.4, 110), new IntakeAuto(-.5).withTimeout(3)),
        Commands.deadline(new intakedeploy().withTimeout(1)),
        Commands.deadline(new EtherStraightCommand(30, .1, .4, 110)),
        Commands.deadline(new EtherStraightCommand(30, 0.5, 0, -.5)),
        Commands.deadline(new EtherStraightCommand(135, 0.5, 0, -0.5)),
        Commands.deadline(new IntakeAuto(.3).withTimeout(1)),
        Commands.deadline(new EtherStraightCommand(20, 0.5, 0, -0.5)));
  }
}
