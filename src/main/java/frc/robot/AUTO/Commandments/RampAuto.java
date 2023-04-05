// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AUTO.Commandments;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AUTO.Commands.MotionMagicAuto;
import frc.robot.AUTO.Commands.MoveUntilPitchChaange;
import frc.robot.AUTO.Commands.MoveUntilPitchChaange.Condition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RampAuto extends SequentialCommandGroup {
  /** Creates a new RampAuto. */
  public RampAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new MoveUntilPitchChaange(15,0,0.2,Condition.LESSTHAN).withTimeout(5),
    new MoveUntilPitchChaange(0,0,0.3,Condition.GREATERTHAN).withTimeout(5),
    new MotionMagicAuto(6, 0).withTimeout(2));
  }
}
