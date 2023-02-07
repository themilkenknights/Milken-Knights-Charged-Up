// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CANID;
import frc.robot.Constants.MKTELE;

public class Tele {
    
    TalonFX telescopeArm;
    private Motor mMotor = Motor.getInstance();

    private Tele()
    {
        telescopeArm = mMotor.motor(CANID.telescopeCANID, MKTELE.TelescopeNeutralMode, 0, MKTELE.pidf, false );
    }

    public static Tele getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public void teleUpdate()
    {
        SmartDashboard.putNumber("Telescope", telescopeArm.getSelectedSensorPosition());
    
    }

    public void telescopePercent(double percent)
    {
        telescopeArm.set(ControlMode.PercentOutput, percent);

    }

    public void zeroVTelescope()
    {
        telescopeArm.setSelectedSensorPosition(0);
    }

    private static class InstanceHolder
    {
        private static final Tele mInstance = new Tele();
    } 
    
}