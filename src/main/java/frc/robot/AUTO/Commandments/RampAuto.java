// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AUTO.Commandments;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AUTO.Commands.BottomIntakeCOMMAND;
import frc.robot.AUTO.Commands.MotionMagicAuto;
import frc.robot.AUTO.Commands.MoveUntilRollChange;
import frc.robot.AUTO.Commands.RampCommand;
import frc.robot.AUTO.Commands.MoveUntilRollChange.Condition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RampAuto extends SequentialCommandGroup {
  /** Creates a new RampAuto. */
  public RampAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new BottomIntakeCOMMAND(.8, 0).withTimeout(2),
        new MoveUntilRollChange(-16, 0, 0.2, Condition.GREATERTHAN).withTimeout(15),
        new MoveUntilRollChange(-15, 0, 0.2, Condition.LESSTHAN).withTimeout(15),
        new MoveUntilRollChange(7, 0, 0.2, Condition.LESSTHAN).withTimeout(15),
        new MoveUntilRollChange(0, 0, 0.2, Condition.GREATERTHAN).withTimeout(15),
        new MotionMagicAuto(15, 0).withTimeout(4),
        new MoveUntilRollChange(9, 0, -0.2, Condition.LESSTHAN).withTimeout(15),
        new MoveUntilRollChange(7, 0, -0.1, Condition.GREATERTHAN).withTimeout(15),
        new MoveUntilRollChange(4, 0, 0.07, Condition.GREATERTHAN).withTimeout(15),
        new RampCommand());
  }
}
