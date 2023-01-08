// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AUTO.DISTANGLE;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SwrvLeftRight extends SequentialCommandGroup {
  /** Creates a new Swrv. */
  public SwrvLeftRight() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    deadline(new Turn(0)).withTimeout(1),
    deadline(new EtherAutoCommand(DISTANGLE.distanceA, DISTANGLE.distance, ((DISTANGLE.angle)),    DISTANGLE.sidePos, DISTANGLE.headingele)).withTimeout(6),
    
    deadline(new Turn(0)).withTimeout(1),
    deadline(new EtherAutoCommand(DISTANGLE.distanceA, DISTANGLE.distance, ((DISTANGLE.angle)),    DISTANGLE.sideCon, DISTANGLE.headingele)).withTimeout(6),

    deadline(new Turn(0)).withTimeout(1),
    deadline(new EtherAutoCommand(DISTANGLE.distanceA, DISTANGLE.distance, ((DISTANGLE.angle)),    DISTANGLE.sidePos, DISTANGLE.headingtwel)).withTimeout(6),
    
    deadline(new Turn(0)).withTimeout(1),
    deadline(new EtherAutoCommand(DISTANGLE.distanceA, DISTANGLE.distance, ((DISTANGLE.angle)),    DISTANGLE.sideCon, DISTANGLE.headingtwel)).withTimeout(6),
    
    deadline(new Turn(0)).withTimeout(1),
    deadline(new EtherAutoCommand(DISTANGLE.distanceA, DISTANGLE.distance, ((DISTANGLE.angle)),    DISTANGLE.sidePos, DISTANGLE.headingthir)).withTimeout(6),
    
    deadline(new Turn(0)).withTimeout(1),
    deadline(new EtherAutoCommand(DISTANGLE.distanceA, DISTANGLE.distance, ((DISTANGLE.angle)),    DISTANGLE.sideCon, DISTANGLE.headingfif)).withTimeout(6),

    deadline(new Turn(0)).withTimeout(1),
    deadline(new EtherAutoCommand(DISTANGLE.distanceA, DISTANGLE.distance, ((DISTANGLE.angle)),    DISTANGLE.sidePos, DISTANGLE.headingfourt)).withTimeout(6),
    
    deadline(new Turn(0)).withTimeout(1),
    deadline(new EtherAutoCommand(DISTANGLE.distanceA, DISTANGLE.distance, ((DISTANGLE.angle)),    DISTANGLE.sideCon, DISTANGLE.headingsixt)).withTimeout(6));
  }
}