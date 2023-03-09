// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AUTO.Commandments;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AUTO.Commands.ArmCommand;
import frc.robot.AUTO.Commands.ClawCommand;
import frc.robot.AUTO.Commands.MotionMagicAuto;
import frc.robot.AUTO.Commands.TelescopingCommand;
import frc.robot.AUTO.Commands.Turn;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SideAuto extends SequentialCommandGroup {
  /** Creates a new SideAuto. */
  public SideAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        Commands.deadline(new MotionMagicAuto(13, 90)),
        Commands.deadline(new ArmCommand(102).withTimeout(3)),
        Commands.deadline(new TelescopingCommand(8200), (new ArmCommand(102))));
        //Commands.deadline(new MotionMagicAuto(2, 0).withTimeout(2)),
        //Commands.deadline(new MotionMagicAuto(9, 270),new ArmCommand(102)),
       // Commands.deadline(new ClawCommand(false).withTimeout(0.5)));
     //  Commands.deadline(new TelescopingCommand(0)),(new MotionMagicAuto(4, 90)),
        //Commands.deadline(new MotionMagicAuto(20, 90) ,new ArmCommand(75)),
        //Commands.deadline(new MotionMagicAuto( 5, 90).withTimeout(7)));   //Commands.deadline(new Turn(180).withTimeout(2)));
    // TelescopingCommand(0).withTimeout(3) ,new ArmCommand(0).withTimeout(3), new
    // ClawCommand(false).withTimeout(2)));
  }
}
