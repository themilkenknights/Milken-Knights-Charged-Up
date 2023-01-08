// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants.NAVX;

/** Add your docs here. */
public class navx {
    private AHRS navx = new AHRS();

    public static navx getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public double getNavxYaw()
    {
        return NAVX.offset + navx.getYaw();
    }

    public AHRS getNavx()
    {
        return navx;
    }

    public void reset()
    {
        navx.zeroYaw();
    }

    private static class InstanceHolder
    {
        private static final navx mInstance = new navx();
    } 
}
