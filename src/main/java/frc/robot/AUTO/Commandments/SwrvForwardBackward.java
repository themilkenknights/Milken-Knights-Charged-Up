// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AUTO.Commandments;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AUTO.Commands.EtherAutoCommand;
import frc.robot.AUTO.Commands.TurnCommand;
import frc.robot.MISC.Constants.AUTO.DISTANGLE;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SwrvForwardBackward extends SequentialCommandGroup {
    /** Creates a new Swrv. */
    public SwrvForwardBackward() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                Commands.deadline(new TurnCommand(0).withTimeout(1)), // forward curve
                Commands.deadline(
                        new EtherAutoCommand(
                                DISTANGLE.distanceA,
                                DISTANGLE.lengthB,
                                DISTANGLE.distance,
                                ((DISTANGLE.angle)),
                                DISTANGLE.headinguno,
                                1))
                        .withTimeout(6),
                Commands.deadline(new TurnCommand(0)).withTimeout(1), // backward curve
                Commands.deadline(
                        new EtherAutoCommand(
                                DISTANGLE.distanceA,
                                DISTANGLE.lengthB,
                                DISTANGLE.distance,
                                ((DISTANGLE.angle)),
                                DISTANGLE.headingdos,
                                1))
                        .withTimeout(6),
                Commands.deadline(new TurnCommand(0)).withTimeout(1),
                Commands.deadline(
                        new EtherAutoCommand(
                                DISTANGLE.distanceA,
                                DISTANGLE.lengthB,
                                DISTANGLE.distance,
                                ((DISTANGLE.angle)),
                                DISTANGLE.headingtres,
                                1))
                        .withTimeout(6),
                Commands.deadline(new TurnCommand(0)).withTimeout(1),
                Commands.deadline(
                        new EtherAutoCommand(
                                DISTANGLE.distanceA,
                                DISTANGLE.lengthB,
                                DISTANGLE.distance,
                                ((DISTANGLE.angle)),
                                DISTANGLE.headingquad,
                                1))
                        .withTimeout(6),
                Commands.deadline(new TurnCommand(0)).withTimeout(1),
                Commands.deadline(
                        new EtherAutoCommand(
                                DISTANGLE.distanceA,
                                DISTANGLE.lengthB,
                                DISTANGLE.distance,
                                ((DISTANGLE.angle)),
                                DISTANGLE.headingsinco,
                                1))
                        .withTimeout(6),
                Commands.deadline(new TurnCommand(0)).withTimeout(1),
                Commands.deadline(
                        new EtherAutoCommand(
                                DISTANGLE.distanceA,
                                DISTANGLE.lengthB,
                                DISTANGLE.distance,
                                ((DISTANGLE.angle)),
                                DISTANGLE.headingsix,
                                1))
                        .withTimeout(6),
                Commands.deadline(new TurnCommand(0)).withTimeout(1),
                Commands.deadline(
                        new EtherAutoCommand(
                                DISTANGLE.distanceA,
                                DISTANGLE.lengthB,
                                DISTANGLE.distance,
                                ((DISTANGLE.angle)),
                                DISTANGLE.headingsev,
                                1))
                        .withTimeout(6),
                Commands.deadline(new TurnCommand(0)).withTimeout(1),
                Commands.deadline(
                        new EtherAutoCommand(
                                DISTANGLE.distanceA,
                                DISTANGLE.lengthB,
                                DISTANGLE.distance,
                                ((DISTANGLE.angle)),
                                DISTANGLE.headingocto,
                                1))
                        .withTimeout(6),
                Commands.deadline(new TurnCommand(0)).withTimeout(1));
    }
}
