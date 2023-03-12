// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AUTO.Commandments;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AUTO.Commands.ArmCommand;
import frc.robot.AUTO.Commands.ClawCommand;
import frc.robot.AUTO.Commands.EtherStraightCommand;
import frc.robot.AUTO.Commands.EtherTurnCommand;
import frc.robot.AUTO.Commands.IntakeAuto;
import frc.robot.AUTO.Commands.MotionMagicAuto;
import frc.robot.AUTO.Commands.TelescopingCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SideAuto extends SequentialCommandGroup {
  /** Creates a new SideAuto. */
  public SideAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
          Commands.deadline(new IntakeAuto().withTimeout(13)),
        //Commands.deadline(new TelescopingCommand(1).withTimeout(1)),
        Commands.deadline(new EtherStraightCommand(155, -0.3,0, 270).withTimeout(20)));
        //  Commands.deadline(new MotionMagicAuto(90, 270).withTimeout(20)));
        //Commands.deadline(new ArmCommand(102).withTimeout(3)),
        //Commands.deadline(new TelescopingCommand(8200), (new ArmCommand(102).withTimeout(2))),
        //Commands.deadline(new MotionMagicAuto(9, 270), new ArmCommand(102).withTimeout(1.5)),
        //Commands.deadline(new ClawCommand(true).withTimeout(1.5)),
        //Commands.deadline(new MotionMagicAuto(155, 270).withTimeout(10)));//,(new ArmCommand(15)), new TelescopingCommand(200), (new ClawCommand(false).withTimeout(1.5))),
        //Commands.deadline(new EtherTurnCommand(90)));
  }
}
