// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AUTO.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.MECHANISMS.MkSwerveTrain;
import frc.robot.MISC.MathFormulas;

public class EtherAutoCommand extends CommandBase {
  private double totalDistance;
  private double thetaTurn;
  private double distanceA;
  private double lengthB;
  private double heading;
  private double side;
  private MkSwerveTrain train = MkSwerveTrain.getInstance();

  public EtherAutoCommand(
      double distanceA, double lengthB, double dist, double ang, double heading, double side) {
    this.totalDistance = dist;
    this.thetaTurn = ang;
    this.distanceA = distanceA;
    this.heading = heading;
    this.lengthB = lengthB;
    this.side = side;
  }

  // youre doing a great job - josh
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    train.startDrive();
    train.setEtherAuto(
        totalDistance, distanceA, MathFormulas.calculateCircleRadius(distanceA, lengthB));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    train.etherAutoUpdate(thetaTurn, heading, side);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    train.stopEverything();
    train.startDrive();
    train.vars.avgDistTest = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return train.isFinished();
  }
}
