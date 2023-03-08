// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AUTO.Commandments;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AUTO.Commands.MotionMagicAuto;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoOne extends SequentialCommandGroup {
  /** Creates a new AutoOne. */
  public AutoOne() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(Commands.deadline(new MotionMagicAuto(10,270)));//Commands.deadline(new EtherStraightCommand(20, 0.3, 0.1, 0.1).withTimeout(6))); //new ArmCommand(90).withTimeout(6), new ClawCommand(true).withTimeout(6)));
  }
}
