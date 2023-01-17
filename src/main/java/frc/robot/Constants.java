// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;


/**All variables that remain constant stored here*/
public final class Constants {

    public static final double kPi = 3.14159265359;
    public static final double[] nullPID = {0,0,0,0};

    public static class MKFALCON 
    {
        public static final int velocityMeasAmount = 26;
        public static final int statusOneMeas = 20;
        public static final int statusTwoMeas = 20;
        public static final double voltComp = 11;
        public static final double oneEncoderRotation = 2048;
    }

    public static class MKDRIVE 
    {
        public static final double kS = 0.1;
        public static final double kA = 0.1;
        public static final double kV = 0.1;

        public static final double maxNativeVelocity = 21600;
        public static final double maxNativeAcceleration = maxNativeVelocity / 8;
        
        public static final double kP = 0.21;
        public static final double kI = 0;
        public static final double kD = 0 * kP;
        public static final double kF = 0;

        public static final double[] pidf = {kP, kI, kD, kF};

        public static final NeutralMode mode = NeutralMode.Brake;

        public static final boolean inverted = false;

        public static final int scurve = 8;

        public static final double greerRatio = 6.75;

        public static final double wheelDiameterInches = 4; 
        public static final double wheelCircumference = wheelDiameterInches * kPi;    
    }

    public static class MKTURN 
    {
        public static final double kP = 0.087;
        public static final double kI = 0;
        public static final double kD = 0.00000001;
        public static final double kF = 0;
        
        public static final double[] pidf = {kP, kI, kD, kF};

        public static final NeutralMode mode = NeutralMode.Coast;

        public static final boolean inverted = true;

        public static final int scurve = 6;

        public static final double greerRatio = 12.8;
    }

    public static class MKCANCODER
    {
        public static final double topLeftOffset = -145.72265625;
        public static final double topRightOffset = 135.615234375;
        public static final double bottomLeftOffset = -117.861328125;
        public static final double bottomRightOffset = 104.765625;
        
        public static final double[] offset = {MKCANCODER.topLeftOffset, MKCANCODER.topRightOffset, MKCANCODER.bottomLeftOffset, MKCANCODER.bottomRightOffset};

        public static final AbsoluteSensorRange range = AbsoluteSensorRange.Signed_PlusMinus180;

        public static final boolean inverted = true;
    }

    public static class MKTRAIN 
    {
        public static final double L = 19.75;
        public static final double W = 25.75;

        public static final double widthInches = 28;
        public static final double heightInches = 28;

        public static final double R = Math.sqrt(Math.pow(L, 2) + Math.pow(W, 2));

        public static final double hP = 0.001, hI = 0.0001, hD = hP * 0.1;

        public static final double speedLimit = 5;
    }

    
    public static class NAVX 
    {
        public static final double offset = 180;
    }
    
    

    public static class MKLIME
    {
        public static final int pipeline = 0;
        public static final double limeHeightInches = 33.5;
        public static final double goalHeightInches = 104;
        public static final double limeAngle = 32;
        public static final double shootTolerance = 3.5;
        public static final double maxTX = 25;
    }

    public static class MKAPRIL
    {
        public static final double kP = 0.1;
        public static final double kI = 0;
        public static final double kD = 0;
    }

    public static class MKBABY
    {
        public static final double fwdBABY = 2;
        public static final double strBABY = 2;
        public static final double rcwBABY = 4;
    }

    public static class CONTROLLERS 
    {
        public static final int driverPort = 0;
        public static final int opPort = 1;

        public static class DriveInput 
        {
            public static final int fwd = 1;
            public static final int str = 0;
            public static final int rcwX = 4;
            public static final int rcwY = 5;
            //1 a, 2 b, 3 x, 4 y
            public static final int resetNavxButton = 4;
            public static final int resetDriveButton = 2;
        }

        public static final int fakeLimelight = 1;

        public static final int topPOV = 0;
        public static final int rightPOV = 90;
        public static final int bottomPOV = 180;
        public static final int leftPOV = 270;
    }

    public static class CANID 
    {
        //drive motors
        public static final int topDriveLeftCANID = 5; 
        public static final int topDriveRightCANID = 7; 
        public static final int bottomDriveLeftCANID = 9; 
        public static final int bottomDriveRightCANID = 3;

        //turn motors
        public static final int topTurnLeftCANID = 6; 
        public static final int topTurnRightCANID = 8; 
        public static final int bottomTurnLeftCANID = 1;
        public static final int bottomTurnRightCANID = 4; 

        //cancoder
        public static final int topTurnLeftCANCoderCANID = 18; 
        public static final int topTurnRightCANCoderCANID = 17; 
        public static final int bottomTurnLeftCANCoderCANID = 15; 
        public static final int bottomTurnRightCANCoderCANID = 16;

        //revh ph
        public static final int revphCANID = 2; //MUST MAKE SURE IT IS ON RIO NOT CANIVORE 
    }

    public static class AUTO
    {

        public static final double measToPredictRatio = 0.823270434926127; //0.93320900560016;

        public static class DISTANGLE 
        {                             
            public static final double distanceA = 80;
            public static final double lengthB = 30;

            public static final int sidePos = 1;
            public static final int sideCon = -1;

            public static final double headinguno = 90;
            public static final double headingdos = -90;
            public static final double headingtres = 90;
            public static final double headingquad = -90;

            public static final double headingsinco = 270;
            public static final double headingsix = -270;
            public static final double headingsev = 270;
            public static final double headingocto = -270;

            public static final double headingnine = 0;
            public static final double headingten = 0;
            public static final double headingele = 360;
            public static final double headingtwel = 180;

            public static final double headingthir = -360;
            public static final double headingfourt = -180;
            public static final double headingfif = -360;
            public static final double headingsixt = -180;

            public static final double distance = MathFormulas.calculateArcOfPath(distanceA, lengthB);
            public static final double angle = MathFormulas.calculateAngleOfPath(distanceA, lengthB);

        }
        //for wpi
        public static final double turnSwerveControlKp = 1;
        public static final double driveSwerveControlKpY = 1;
        public static final double driveSwerveControlKpX = 1;

        public static final double heightMeters = MathFormulas.inchesToMeters(MKTRAIN.heightInches / 2);
        public static final double widthMeters = MathFormulas.inchesToMeters(MKTRAIN.widthInches / 2);

        public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
        new Translation2d(heightMeters, widthMeters),
        new Translation2d(heightMeters, -widthMeters),
        new Translation2d(-heightMeters, widthMeters),
        new Translation2d(-heightMeters, -widthMeters));
      
        
        //actual drive module stats
        public static final double maxModuleTurnVelo = kPi;
        public static final double maxModuleTurnAccel = kPi;
        
        //actual drive module stats
        public static final double maxModuleDriveVelo = 1;
        public static final double maxModuleDriveAccel = 1;
        

        //for turning constraints
        public static final double maxAutoTurnVelo = kPi;
        public static final double maxAutoTurnAccel = kPi;
        
        //for trajectory config
        public static final double maxAutoDriveVelo = 1; //2;
        public static final double maxAutoDriveAccel = 1; //2;


        public static final double maxDriveVelo = 1;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            maxAutoTurnVelo, maxAutoTurnAccel);
    }


    public static class ODO 
    {
        public static final double goalXInches = 120;
        public static final double goalYInches = 120;
        public static final double goalRadius = 60;
    }

    public static class LIGHTS 
    {
        public static final int PWMPORT = 0; 
        public static final int bufferNum = 151; 
        public static final int MaxRGBValue = 60;
    }

    public static class LOGS
    {
      public static final int maxSizeThreshold = 100000;
    }
}
