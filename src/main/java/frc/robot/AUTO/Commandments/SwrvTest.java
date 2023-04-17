package frc.robot.AUTO.Commandments;
// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AUTO.Commands.EtherAutoCommand;
import frc.robot.AUTO.Commands.TurnCommand;
import frc.robot.MISC.Constants.AUTO.DISTANGLE;
import frc.robot.MISC.MathFormulas;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SwrvTest extends SequentialCommandGroup {
  /** Creates a new SwrvTest. */
  public SwrvTest() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        Commands.deadline(new TurnCommand(0)).withTimeout(1),
        Commands.deadline(
                new EtherAutoCommand(
                    DISTANGLE.distanceA,
                    DISTANGLE.lengthB,
                    DISTANGLE.distance,
                    ((DISTANGLE.angle)),
                    360,
                    1))
            .withTimeout(6),
        Commands.deadline(new TurnCommand(0)).withTimeout(1),
        Commands.deadline(
                new EtherAutoCommand(
                    50,
                    20,
                    MathFormulas.calculateArcOfPath(50, 20),
                    MathFormulas.calculateAngleOfPath(50, 20),
                    -90,
                    -1))
            .withTimeout(6),
        Commands.deadline(new TurnCommand(0)).withTimeout(1),
        Commands.deadline(
                new EtherAutoCommand(
                    DISTANGLE.distanceA,
                    DISTANGLE.lengthB,
                    DISTANGLE.distance,
                    ((DISTANGLE.angle)),
                    -360,
                    -1))
            .withTimeout(6),
        Commands.deadline(new TurnCommand(0)).withTimeout(1),
        Commands.deadline(
                new EtherAutoCommand(
                    50,
                    20,
                    MathFormulas.calculateArcOfPath(50, 20),
                    MathFormulas.calculateAngleOfPath(50, 20),
                    90,
                    -1))
            .withTimeout(6),
        Commands.deadline(new TurnCommand(0)).withTimeout(1),
        Commands.deadline(
                new EtherAutoCommand(
                    50,
                    20,
                    MathFormulas.calculateArcOfPath(50, 20),
                    MathFormulas.calculateAngleOfPath(50, 20),
                    -90,
                    1))
            .withTimeout(6),
        Commands.deadline(new TurnCommand(0)).withTimeout(1),
        Commands.deadline(
                new EtherAutoCommand(
                    DISTANGLE.distanceA,
                    DISTANGLE.lengthB,
                    DISTANGLE.distance,
                    ((DISTANGLE.angle)),
                    180,
                    -1))
            .withTimeout(6),
        Commands.deadline(new TurnCommand(0)).withTimeout(1),
        Commands.deadline(
                new EtherAutoCommand(
                    50,
                    20,
                    MathFormulas.calculateArcOfPath(50, 20),
                    MathFormulas.calculateAngleOfPath(50, 20),
                    -90,
                    -1))
            .withTimeout(6),
        Commands.deadline(new TurnCommand(0)).withTimeout(1),
        Commands.deadline(
                new EtherAutoCommand(
                    DISTANGLE.distanceA,
                    DISTANGLE.lengthB,
                    DISTANGLE.distance,
                    ((DISTANGLE.angle)),
                    180,
                    -1))
            .withTimeout(6));
  }
}
