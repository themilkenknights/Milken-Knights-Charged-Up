// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AUTO.Commandments;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AUTO.Commands.EtherStraightCommand;
import frc.robot.AUTO.Commands.EtherTurnCommand;
import frc.robot.AUTO.Commands.IntakeAuto;
import frc.robot.AUTO.Commands.MotionMagicAuto;
import frc.robot.AUTO.Commands.intakedeploy;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SideAuto extends SequentialCommandGroup {
  /** Creates a new SideAuto. */
  public SideAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        Commands.deadline(new IntakeAuto(.8).withTimeout(1)),
        Commands.deadline(new EtherStraightCommand(185, -0.5,0, 110)),
        Commands.deadline(new intakedeploy().withTimeout(1)),
        Commands.deadline(new EtherStraightCommand(50, -.1, -.4, 110), new IntakeAuto(-.5).withTimeout(2)),
        Commands.deadline(new intakedeploy().withTimeout(1)),
        Commands.deadline(new EtherStraightCommand(30, .1, .4, 110)),
        Commands.deadline(new EtherStraightCommand(160, 0.5,0, 0)),
        Commands.deadline(new IntakeAuto(.7).withTimeout(1)));


  }
}
