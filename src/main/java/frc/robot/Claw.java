// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants.CANID;

public class Claw {
    private Solenoid Claw;

    private Claw()
    {
        Claw = new Solenoid(PneumaticsModuleType.REVPH, CANID.CLAWCANID);
    }

    public static Claw getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public void ClawSet(boolean state)
    {
        Claw.set(state);
    }

    public void clawToggle()
    {
        Claw.toggle();
    }

    public boolean getClawState()
    {
        return Claw.get();
    }

    private static class InstanceHolder
    {
        private static final Claw mInstance = new Claw();
    } 
}
