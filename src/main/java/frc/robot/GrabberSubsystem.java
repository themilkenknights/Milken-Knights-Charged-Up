
// Author: UMN Robotics Ri3d
// Last Updated : January 2023

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANID;

import frc.robot.Constants;

public class GrabberSubsystem extends SubsystemBase {
  private boolean isExtended; // This variable keeps track of whether the grabber piston is currently extended or not
  private Solenoid grabberPiston; // A double solenoid takes up two PCM channels

  /** Subsystem for controlling the Grabber (Picks up both cubes and cones) which works via a pneumatic piston */
  public GrabberSubsystem() {
    grabberPiston = new Solenoid(CANID.revphCANID, PneumaticsModuleType.REVPH, CANID.CLAWPORT);
    isExtended = false; // The grabber piston will be initially extended (the grabber will be holding a preloaded cube/cone)
  }

  /* Methods for controlling the state of the DoubleSolenoid */
  public void extend() {
    isExtended = true; // Initial position of the grabber piston
  }
  public void retract() {
    isExtended = false;
  }
  public void toggle() { // Toggle the state of the grabber
    if (isExtended) {
      retract();
    } else {
      extend();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}