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
public class SwrvLeftRight extends SequentialCommandGroup {
    /** Creates a new Swrv. */
    public SwrvLeftRight() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                Commands.deadline(new TurnCommand(0)).withTimeout(1),
                Commands.deadline(
                        new EtherAutoCommand(
                                DISTANGLE.distanceA,
                                DISTANGLE.distance,
                                ((DISTANGLE.angle)),
                                DISTANGLE.sidePos,
                                DISTANGLE.headingele,
                                1))
                        .withTimeout(6),
                Commands.deadline(new TurnCommand(0)).withTimeout(1),
                Commands.deadline(
                        new EtherAutoCommand(
                                DISTANGLE.distanceA,
                                DISTANGLE.distance,
                                ((DISTANGLE.angle)),
                                DISTANGLE.sideCon,
                                DISTANGLE.headingele,
                                1))
                        .withTimeout(6),
                Commands.deadline(new TurnCommand(0)).withTimeout(1),
                Commands.deadline(
                        new EtherAutoCommand(
                                DISTANGLE.distanceA,
                                DISTANGLE.distance,
                                ((DISTANGLE.angle)),
                                DISTANGLE.sidePos,
                                DISTANGLE.headingtwel,
                                1))
                        .withTimeout(6),
                Commands.deadline(new TurnCommand(0)).withTimeout(1),
                Commands.deadline(
                        new EtherAutoCommand(
                                DISTANGLE.distanceA,
                                DISTANGLE.distance,
                                ((DISTANGLE.angle)),
                                DISTANGLE.sideCon,
                                DISTANGLE.headingtwel,
                                1))
                        .withTimeout(6),
                Commands.deadline(new TurnCommand(0)).withTimeout(1),
                Commands.deadline(
                        new EtherAutoCommand(
                                DISTANGLE.distanceA,
                                DISTANGLE.distance,
                                ((DISTANGLE.angle)),
                                DISTANGLE.sidePos,
                                DISTANGLE.headingthir,
                                1))
                        .withTimeout(6),
                Commands.deadline(new TurnCommand(0)).withTimeout(1),
                Commands.deadline(
                        new EtherAutoCommand(
                                DISTANGLE.distanceA,
                                DISTANGLE.distance,
                                ((DISTANGLE.angle)),
                                DISTANGLE.sideCon,
                                DISTANGLE.headingfif,
                                1))
                        .withTimeout(6),
                Commands.deadline(new TurnCommand(0)).withTimeout(1),
                Commands.deadline(
                        new EtherAutoCommand(
                                DISTANGLE.distanceA,
                                DISTANGLE.distance,
                                ((DISTANGLE.angle)),
                                DISTANGLE.sidePos,
                                DISTANGLE.headingfourt,
                                1))
                        .withTimeout(6),
                Commands.deadline(new TurnCommand(0)).withTimeout(1),
                Commands.deadline(
                        new EtherAutoCommand(
                                DISTANGLE.distanceA,
                                DISTANGLE.distance,
                                ((DISTANGLE.angle)),
                                DISTANGLE.sideCon,
                                DISTANGLE.headingsixt,
                                1))
                        .withTimeout(6));
    }
}
