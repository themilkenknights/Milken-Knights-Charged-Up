// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CAMERA.AprilTags;
import frc.robot.MECHANISMS.MkSwerveTrain;
import frc.robot.MISC.Constants.CONTROLLERS.DriveInput;
import frc.robot.MISC.Lights;
import frc.robot.MISC.MathFormulas;
import frc.robot.MISC.Odometry;
import frc.robot.MISC.pigeon;

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
      lightMode = 0;
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
      itsreal = false;
  private boolean isRCWrunningWithpig = false;
  private AprilTags april = AprilTags.getInstance();
  private Timer turntesttimer = new Timer();
  private Lights mLights = Lights.getInstance();
  private Timer turntesttimertwo = new Timer();
  private double count = 0;
  private Timer aprilTimer = new Timer();
  private double x, y, rot;
  private Timer pitcheck = new Timer();

  public static SupaStruct getInstance() {
    return InstanceHolder.mInstance;
  }

  public boolean getAprilEnabled() {
    return rbbutton;
  }

  public void initTele() {
    lightMode = 0;
    aprilTimer.start();
    pigRotate = pigeon.getInstance().getPigYaw();
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

    train.updateSwerve();

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
      /*if(rcw < 0.1 && (fwd < 0.1 && str < 0.1))
      {
        rcw = 0;
      }*/
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
    } else if (rbbutton) {
      //Ramp.getInstance().rampMove(0);
    } else if ((fwd != 0 || str != 0 || rcw != 0)) {
      train.etherSwerve(fwd, str, rcw, ControlMode.PercentOutput); // +,-,+
    } /*else if (pigeon.getInstance().getPigPitch() > PIGEON.pitchThreshold) {
      //fwd = train.antiTip()[1];
      //str = train.antiTip()[0];
      //train.etherSwerve(fwd, -str, 0, ControlMode.PercentOutput);
    */ else {
      train.stopEverything();
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
    } else if (lightMode == 3) {
      mLights.GROUND();
      SmartDashboard.putString("color", "ground");
    }

    // --------------------------------------------------------------------//
    // SMARTDASHBOARD
    // --------------------------------------------------------------------//

  }

  public void teleopDisabled() {
    resetpig = false;
    resetDrive = false;
    xbutton = false;
    ybutton = false;
    pov = false;
    itsreal = false;
    turntesttimer.stop();
    turntesttimer.reset();
    pitcheck.stop();
    pitcheck.reset();
  }

  public void initTest() {
    /*
     * train.vars.avgDistTest = 0;
     * turntesttimer.stop();
     * turntesttimer.reset();
     * turntesttimertwo.stop();
     * turntesttimertwo.reset();
     */
    train.startTrain();
    pitcheck.start();
  }

  // measured over predicted * predicted
  public void updateTest() {
    if(pitcheck.get() < 5)
    {
      //train.setModuleDrive(null, fwdSignum, str, fwd, count);
    }
    /*
     * double fwd = 0;
     * double rcw = 0;
     * if (xbox.getAButtonPressed()) {
     * turntesttimer.start();
     * }
     * if (turntesttimer.get() > 0.00000000000000001 && turntesttimer.get() < 5) {
     * fwd = 0.3;
     * }
     * if (xbox.getRawAxis(4) > 0.1
     * && (turntesttimer.get() > 0.00000000000000001 && turntesttimer.get() < 5)) {
     * rcw = 0.5;
     * count++;
     * }
     * 
     * if (fwd == 0.3 || rcw == 0.5) {
     * train.etherSwerve(fwd, 0, rcw / 2, ControlMode.PercentOutput);
     * train.etherRCWFinder(fwd, 0, 0);
     * } else {
     * train.stopEverything();
     * }
     * 
     * // SmartDashboard.putNumber("count", count);
     * // SmartDashboard.putNumber(
     * // "meastopredictratio", train.vars.avgDistInches / train.vars.avgDistTest);
     * // SmartDashboard.putNumber("delta", train.vars.avgDistTest);
     */
  }

  private static class InstanceHolder {
    private static final SupaStruct mInstance = new SupaStruct();
  }
}
