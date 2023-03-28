// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CAMERA.AprilTags;
import frc.robot.MECHANISMS.ARM.Arm;
import frc.robot.MECHANISMS.ARM.Claw;
import frc.robot.MECHANISMS.ARM.Wrist;
import frc.robot.MECHANISMS.Intake;
import frc.robot.MECHANISMS.MkSwerveTrain;
import frc.robot.MISC.Constants.CONTROLLERS.DriveInput;
import frc.robot.MISC.Constants.MKTELE;
import frc.robot.MISC.Constants.PIGEON;
import frc.robot.MISC.Lights;
import frc.robot.MISC.MathFormulas;
import frc.robot.MISC.Odometry;
import frc.robot.MISC.pigeon;
import java.util.Map;

/** Robot stuff in here */
public class SupaStruct {

  private XboxController xbox = new XboxController(0);
  private XboxController xboxOP = new XboxController(1);
  private double fwd,
      fwdSignum,
      str,
      strSignum,
      leftjoy,
      rcw,
      rcwX,
      rcwY,
      inverseTanAngleOG,
      inverseTanAngleDrive,
      povValue,
      pigRotate = 0,
      lightMode = 0,
      sliderArm;
  private MkSwerveTrain train = MkSwerveTrain.getInstance();
  private Odometry odo = Odometry.getInstance();

  private boolean resetpig,
      dpadup,
      dpaddown,
      resetTurn,
      resetDrive,
      xbutton,
      ybutton,
      rbbutton,
      bbutton,
      rbbutton2,
      lbbutton2,
      lbbutton,
      abutton,
      ltrigger,
      rtrigger,
      xbutton2,
      ybutton2,
      abutton2,
      bbutton2,
      dpadup2,
      dpaddown2,
      ltrigger2,
      rtrigger2,
      dpadleft2,
      dpadright2,
      dpadleft,
      dpadright,
      toggleLightsPressed = false,
      pov, /* povToggled, */
      itsreal = false,
      resetDoneDiddlyDoneTELE = false,
      resetDoneDiddlyDoneWRIST = false,
      toggleConeOn,
      toggleCubeOn = false,
      toggleArmHighOn = false,
      toggleArmMidOn = false,
      toggleArmLowOn = false,
      toggleArmStowOn = false,
      manualMoveWrist = false;
  private boolean isRCWrunningWithpig = false;
  private AprilTags april = AprilTags.getInstance();
  private Intake intake = Intake.getInstance();
  private Wrist wrist = Wrist.getInstance();
  private Timer turntesttimer = new Timer();
  private Claw claw = Claw.getInstance();
  private Arm arm = Arm.getInstance();
  private Lights mLights = Lights.getInstance();
  private Timer turntesttimertwo = new Timer();
  private double count = 0;
  private Timer aprilTimer = new Timer();

  private ShuffleboardTab tab = Shuffleboard.getTab("slida");
  private GenericEntry slidaa;
  private double x, y, rot;

  public static SupaStruct getInstance() {
    return InstanceHolder.mInstance;
  }

  public boolean getAprilEnabled() {
    return rbbutton;
  }

  public void initTele() {
    aprilTimer.start();
    try {
      slidaa = tab.add("slidaa", 1)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", -1, "max", 1))
          .getEntry();

    } catch (Exception e) {
      System.out.println(
          "fuck you if the slider is there just have an in built system to say 'oh look its FUCKING THERE' i swear to god"
              + e);
    }
    pigRotate = pigeon.getInstance().getPigYaw();
    arm.setArmToCanCoder();
  }

  public void updateTele() {

    // --------------------------------------------------------------------//
    // UPDATES
    // --------------------------------------------------------------------//
    if (xbox.getRawButton(9)) {
      april.updateApril();
      aprilTimer.restart();
      april.aprilSmartDashboard();
      x = april.getAxis("x");
      y = april.getAxis("y");
      // yaw = april.getAxis("yaw");
      rot = april.getAxis("r");
    }

    updateLightsToggle();
    train.updateSwerve();
    wrist.updateZeroWristMotor();
    arm.updateZeroTelescopeMotor();
    arm.updateSmartdashboard();

    // --------------------------------------------------------------------//
    // VARIABLES
    // --------------------------------------------------------------------//

    fwd = (xbox.getRawAxis(DriveInput.fwd) - 0.1) / (1 - 0.1);
    fwdSignum = Math.signum(fwd) * -1;
    str = (xbox.getRawAxis(DriveInput.str) - 0.1) / (1 - 0.1);
    strSignum = Math.signum(str) * -1;
    rcwY = (xbox.getRawAxis(DriveInput.rcwY) - 0.1) / (1 - 0.1);
    // Todo see if making this x breaks it
    rcwX = (xbox.getRawAxis(DriveInput.rcwX) - 0.1) / (1 - 0.1);
    rcw = rcwX;

    // DRIVER
    xbutton = xbox.getXButton();
    abutton = xbox.getAButtonPressed();
    rbbutton = xbox.getRightBumper();
    ybutton = xbox.getYButton();
    bbutton = xbox.getBButton();
    lbbutton = xbox.getLeftBumper();
    dpaddown = xbox.getPOV() == 180;
    dpadleft = xboxOP.getPOV() == 90;
    dpadright = xboxOP.getPOV() == 270;
    dpadup = xbox.getPOV() == 0;
    ltrigger = Math.abs(xbox.getRawAxis(2)) > 0.1;
    rtrigger = Math.abs(xbox.getRawAxis(3)) > 0.1;

    // OP
    xbutton2 = xboxOP.getXButton();
    abutton2 = xboxOP.getAButton();
    rbbutton2 = xboxOP.getRightBumper();
    ybutton2 = xboxOP.getYButton();
    bbutton2 = xboxOP.getBButton();
    lbbutton2 = xboxOP.getLeftBumper();
    dpaddown2 = xboxOP.getPOV() == 180;
    dpadup2 = xboxOP.getPOV() == 0;
    dpadleft2 = xboxOP.getPOV() == 90;
    dpadright2 = xboxOP.getPOV() == 270;
    ltrigger2 = Math.abs(xboxOP.getRawAxis(2)) > 0.1;
    rtrigger2 = Math.abs(xboxOP.getRawAxis(3)) > 0.1;

    pov = xbox.getPOV() != -1;

    inverseTanAngleDrive = ((((((Math.toDegrees(Math.atan(fwd / str)) + 360)) + (MathFormulas.signumV4(str))) % 360)
        - MathFormulas.signumAngleEdition(str, fwd))
        + 360)
        % 360;

    // --------------------------------------------------------------------//
    // PIGEON RESET
    // --------------------------------------------------------------------//

    if (ybutton) {
      pigeon.getInstance().reset();
      povValue = 00;
      inverseTanAngleOG = 0;
      train.vars.avgDistTest = 0;
      train.vars.avgDistInches = 0;
      train.startTurn();
      train.startDrive();
    }

    if (pov) {
      rcw = train.moveToAngy(xbox.getPOV());
    }

    // --------------------------------------------------------------------//
    // DRIVE STATEMENTS
    // --------------------------------------------------------------------//

    if (Math.abs(xbox.getRawAxis(DriveInput.rcwY)) < 0.1
        && Math.abs(xbox.getRawAxis(DriveInput.rcwX)) < 0.1
        && !pov) { // && xbox.getPOV() == -1) {
      rcw = 0;
    }

    if (Math.abs(xbox.getRawAxis(DriveInput.rcwY)) < 0.1) {
      rcwY = 0;
    }
    if (Math.abs(xbox.getRawAxis(DriveInput.rcwX)) < 0.1) {
      rcwX = 0;
    }

    if (Math.abs(xbox.getRawAxis(DriveInput.fwd)) < 0.1) {
      fwd = 0;
    }
    if (Math.abs(xbox.getRawAxis(DriveInput.str)) < 0.1) {
      str = 0;
    }

    if (xbutton) {
      april.alignToTag();

    } else if ((fwd != 0 || str != 0 || rcw != 0)) {
      train.etherSwerve(fwd, str, -rcw / 2, ControlMode.PercentOutput); // +,-,+
    } else if (pigeon.getInstance().getPigPitch() > PIGEON.pitchThreshold) {
      fwd = train.antiTip()[1];
      str = train.antiTip()[0];
      train.etherSwerve(fwd, -str, 0, ControlMode.PercentOutput);
    } else {
      train.stopEverything();
    }

    // --------------------------------------------------------------------//
    // INTAKE
    // --------------------------------------------------------------------//

    if (rtrigger) {
      intake.rollerSet(-1);

    } else if (ltrigger) {
      intake.rollerSet(1);

    } else {
      intake.rollerSet(0);
    }
    if (abutton) {
      intake.toggle();
    }

    /*
     * RT - Cone Select
     * LT - Cube Select
     * RB - Cone In (Cube Out)
     * LB - Cone Out (Cube In)
     * Y - High --
     * A - Low--
     * B - Mid--
     * X - Stow--
     * Dpad left right for manual extension retraction-- NEW NOW ARM
     * Dpad up down for manual rotation up down--
     */

    // --------------------------------------------------------------------//
    // WRIST
    // --------------------------------------------------------------------//

    if (dpadup2) {
      wrist.moveWristMotor(-.3);
      manualMoveWrist = true;
    } else if (dpaddown2) {
      wrist.moveWristMotor(.3);
      manualMoveWrist = true;
    } else if (resetDoneDiddlyDoneWRIST) {
      wrist.moveWristMotor(0);
      // TODO see if this setting here fucks it all up
      // manualMoveWrist = true;
    }

    if (rbbutton2) {
      if (toggleConeOn) {
        wrist.moveWristRoller(-1);
        // run rollers direction 1
      } else if (toggleCubeOn) {
        wrist.moveWristRoller(1);
        // run rollers direction 2
      }
    } else if (lbbutton2) {
      if (toggleConeOn) {
        wrist.moveWristRoller(1);
        // run rollers direction 2
      } else if (toggleCubeOn) {
        wrist.moveWristRoller(-1);
        // run rollers direction 1
      }
    } else {
      wrist.moveWristRoller(0);
    }

    // --------------------------------------------------------------------//
    // ARM
    // --------------------------------------------------------------------//

    if (dpadleft2 && !dpadright2) {
      arm.moveArm(-0.16, -0.16);

    } else if (!dpadleft2 && dpadright2) {
      arm.moveArm(0.16, 0.16);
    }

    if (lbbutton) {
      arm.moveArm(sliderArm, sliderArm);
    }

    if (!lbbutton && !dpadleft2 && !dpadright2) {
      arm.moveArm(0, 0);
    }

    // --------------------------------------------------------------------//
    // TELESCOPE
    // --------------------------------------------------------------------//

    if (!rtrigger2 && ltrigger2 && arm.getTelescope() > MKTELE.minNativePositionTelescope) {
      arm.moveTele(-.4);
    } else if (rtrigger2 && !ltrigger2 && arm.getTelescope() < MKTELE.maxNativePositionTelescope) {
      arm.moveTele(.4);
    } else if(resetDoneDiddlyDoneTELE){
      arm.moveTele(0);
    }

    if (xbox.getRawButton(7)) {
      arm.setTelescopeEncoder(MKTELE.minNativePositionTelescope / MKTELE.greerRatio);
    } else if (xbox.getRawButton(8)) {
      arm.setTelescopeEncoder(MKTELE.maxNativePositionTelescope / MKTELE.greerRatio);
    }

    /*
     * if (dpadleft2) {
     * odo.reset();
     * }
     * if (dpadright2) {
     * odo.resetToPose2D(x, y, rot);
     * }
     */

    // --------------------------------------------------------------------//
    // COMPETITION TOGGLES
    // --------------------------------------------------------------------//

    if (xboxOP.getRawButton(8)) {
      toggleConeOn = true;
      toggleCubeOn = false;
      manualMoveWrist = false;

    } else if (xboxOP.getRawButton(7)) {
      toggleConeOn = false;
      toggleCubeOn = true;
      manualMoveWrist = false;
    }

    if (xboxOP.getRawButton(9)) {
      toggleConeOn = false;
      toggleCubeOn = false;
      toggleArmHighOn = false;
      toggleArmMidOn = false;
      toggleArmLowOn = false;
      toggleArmStowOn = false;
      manualMoveWrist = true;
    }

    if (ybutton2) {
      toggleArmHighOn = true;
      toggleArmMidOn = false;
      toggleArmLowOn = false;
      toggleArmStowOn = false;
      manualMoveWrist = false;
    } else if (bbutton2) {
      toggleArmHighOn = false;
      toggleArmMidOn = true;
      toggleArmLowOn = false;
      toggleArmStowOn = false;
      manualMoveWrist = false;
    } else if (xbutton2) {
      toggleArmHighOn = false;
      toggleArmMidOn = false;
      toggleArmLowOn = true;
      toggleArmStowOn = false;
      manualMoveWrist = false;
    } else if (abutton2) {
      toggleArmHighOn = false;
      toggleArmMidOn = false;
      toggleArmLowOn = false;
      toggleArmStowOn = true;
      manualMoveWrist = false;
    }

    if (toggleArmHighOn) {
      if (toggleConeOn) {
        arm.pidArm(116);

        if (!manualMoveWrist) {
          wrist.moveWristPID(250);
        }
      } else if (toggleCubeOn) {
        arm.pidArm(93.5);

        if (!manualMoveWrist) {
          wrist.moveWristPID(116.5);
        }
      }

    } else if (toggleArmMidOn) {
      // toggle HIGH AND HP
      if (toggleConeOn) {
        arm.pidArm(96);
        if (!manualMoveWrist) {
          wrist.moveWristPID(230);
        }
      } else if (toggleCubeOn) {
        arm.pidArm(75);
        if (!manualMoveWrist) {
          wrist.moveWristPID(25);
        }
      }

    } else if (toggleArmLowOn) {
      if (toggleConeOn) {
        arm.pidArm(37);
        arm.pidTelescope(200);
        if (!manualMoveWrist) {
          wrist.moveWristPID(200);
        }
      } else if (toggleCubeOn) {
        arm.pidArm(40);
        arm.pidTelescope(5000);
        if (!manualMoveWrist) {
          wrist.moveWristPID(50);
        }
      }

    } else if (toggleArmStowOn) {
      if (toggleConeOn) {
        arm.pidArm(20);
        arm.pidTelescope(0);
        if (!manualMoveWrist) {
          wrist.moveWristPID(0);
        }
      } else if (toggleCubeOn) {
        arm.pidArm(20);
        arm.pidTelescope(0);
        if (!manualMoveWrist) {
          wrist.moveWristPID(0);
        }
      }
    }

    // --------------------------------------------------------------------//
    // LEDS
    // --------------------------------------------------------------------//

    if (lightMode == 0) {
      mLights.off();
      SmartDashboard.putString("color", "none");
    } else if (lightMode == 1) {
      mLights.CONE();
      SmartDashboard.putString("color", "cone");
    } else if (lightMode == 2) {
      mLights.CUBE();
      SmartDashboard.putString("color", "cube");
    }

    // --------------------------------------------------------------------//
    // SMARTDASHBOARD
    // --------------------------------------------------------------------//

    // SmartDashboard.putNumber("rcw", xbox.getPOV());
    // SmartDashboard.putNumber("pov", xbox.getPOV());
    // SmartDashboard.putNumber("up", wrist.getWristMotorGudAngle(MODE.up));
    // SmartDashboard.putNumber("down", wrist.getWristMotorGudAngle(MODE.down));
    // SmartDashboard.putNumber("out", wrist.getWristMotorGudAngle(MODE.out));
    // SmartDashboard.putNumber("getwrist", wrist.getWristNative());
    // SmartDashboard.putNumber("neo 550",
    // MathFormulas.sparkToDegrees(wrist.getWristNative()));
    // SmartDashboard.putNumber("setpoint pid", wrist.getWristMotorSpeed());
    // SmartDashboard.putNumber("degreetospark", MathFormulas.degreesToSpark(100));
    // SmartDashboard.putBoolean("togglearmhign", toggleArmHighOn);
    // SmartDashboard.putBoolean("togglearmlow", toggleArmLowOn);
    // SmartDashboard.putBoolean("togglemidarm", toggleArmMidOn);
    // SmartDashboard.putBoolean("togglearmstow", toggleArmStowOn);
    // SmartDashboard.putBoolean("cone", toggleConeOn);
    // SmartDashboard.putBoolean("cube", toggleCubeOn);
    // SmartDashboard.putNumber("Armangle", arm.getArmDegrees());
    // SmartDashboard.putNumber("armcan", arm.getArmCanCoder());
    // SmartDashboard.putBoolean("manual", manualMoveWrist);

    sliderArm = slidaa.getDouble(0);
  }

  public void updateLightsToggle() {
    if (bbutton) {
      if (!toggleLightsPressed) {
        lightMode++;
        lightMode = lightMode % 3;
        toggleLightsPressed = true;
      }
    } else {
      toggleLightsPressed = false;
    }
  }

  public void setStartupWristToTrue() {
    resetDoneDiddlyDoneWRIST = true;
  }

  public void setStartupTelescopeToTrue() {
    resetDoneDiddlyDoneTELE = true;
  }

  public void teleopDisabled() {
    resetpig = false;
    resetDrive = false;
    xbutton = false;
    ybutton = false;
    pov = false;
    itsreal = false;
    toggleConeOn = false;
    toggleCubeOn = false;
    toggleArmHighOn = false;
    toggleArmMidOn = false;
    toggleArmLowOn = false;
    toggleArmStowOn = false;
    turntesttimer.stop();
    turntesttimer.reset();
    try {
      slidaa.close();
    } catch (Exception e) {
      System.out.println("end");
    }
  }

  public void initTest() {
    train.vars.avgDistTest = 0;
    turntesttimer.stop();
    turntesttimer.reset();
    turntesttimertwo.stop();
    turntesttimertwo.reset();
    train.startTrain();
  }

  // measured over predicted * predicted
  public void updateTest() {
    double fwd = 0;
    double rcw = 0;
    if (xbox.getAButtonPressed()) {
      turntesttimer.start();
    }
    if (turntesttimer.get() > 0.00000000000000001 && turntesttimer.get() < 5) {
      fwd = 0.3;
    }
    if (xbox.getRawAxis(4) > 0.1
        && (turntesttimer.get() > 0.00000000000000001 && turntesttimer.get() < 5)) {
      rcw = 0.5;
      count++;
    }

    if (fwd == 0.3 || rcw == 0.5) {
      train.etherSwerve(fwd, 0, rcw / 2, ControlMode.PercentOutput);
      train.etherRCWFinder(fwd, 0, 0);
    } else {
      train.stopEverything();
    }

    // SmartDashboard.putNumber("count", count);
    // SmartDashboard.putNumber(
    // "meastopredictratio", train.vars.avgDistInches / train.vars.avgDistTest);
    // SmartDashboard.putNumber("delta", train.vars.avgDistTest);
  }

  private static class InstanceHolder {
    private static final SupaStruct mInstance = new SupaStruct();
  }
}
