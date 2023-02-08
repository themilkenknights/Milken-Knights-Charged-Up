// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CONTROLLERS.DriveInput;
import frc.robot.Constants.MKBABY;
import frc.robot.Constants.MKTELE;

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
      navxRotate = 0;
  private MkSwerveTrain train = MkSwerveTrain.getInstance();

  private boolean resetNavx,
      dpadup,
      dpaddown,
      intakeOvveride,
      ballEnterOvverride,
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
      pov, /* povToggled, */
      itsreal = false;
  private boolean isRCWrunningWithNavx = false;
  private AprilTags april = AprilTags.getInstance();
  private Intake intake = Intake.getInstance();
  private Timer turntesttimer = new Timer();
  private Claw claw = Claw.getInstance();
  private Arm arm = Arm.getInstance();
  private Timer turntesttimertwo = new Timer();
  private double count = 0;

  public static SupaStruct getInstance() {
    return InstanceHolder.mInstance;
  }

  public void initTele() {
    navxRotate = navx.getInstance().getNavxYaw();
  }

  public void updateTele() {
    // TODO seperate all smartdashboard update from main update in update function
    // in all classes
    // --------------------------------------------------------------------//
    // UPDATES
    // --------------------------------------------------------------------//
    train.updateSwerve();
    april.updateApril();

    // lime.updateSensors();
    april.aprilSmartDashboard();
    arm.updateSmartdashboard();

    // lime.limeSmartDashboard();

    // --------------------------------------------------------------------//
    // VARIABLES
    // --------------------------------------------------------------------//

    fwd = (xbox.getRawAxis(DriveInput.fwd) - 0.1) / (1 - 0.1);
    fwdSignum = Math.signum(fwd) * -1;
    str = (xbox.getRawAxis(DriveInput.str) - 0.1) / (1 - 0.1);
    strSignum = Math.signum(str) * -1;
    rcw = (xbox.getRawAxis(DriveInput.rcwY) - 0.1) / (1 - 0.1);
    rcwY = rcw;
    // Todo see if making this x breaks it
    rcwX = (xbox.getRawAxis(DriveInput.rcwX) - 0.1) / (1 - 0.1);
    resetNavx = xbox.getRawButton(DriveInput.resetNavxButton);
    resetDrive = xbox.getRawButton(DriveInput.resetDriveButton);
    xbutton = xbox.getXButtonPressed();
    abutton = xbox.getAButtonPressed();
    rbbutton = xbox.getRightBumper();
    bbutton = xbox.getBButtonPressed();
    lbbutton = xbox.getLeftBumper();
    dpaddown = xbox.getPOV() == 180;
    dpadup = xbox.getPOV() == 0;
    SmartDashboard.putNumber("pov", xbox.getPOV());
    ltrigger = Math.abs(xbox.getRawAxis(2)) > 0.1;
    rtrigger = Math.abs(xbox.getRawAxis(3)) > 0.1;
    
    pov = xbox.getPOV() != -1;

    // i dont remember how i got this lol

    inverseTanAngleDrive = ((((((Math.toDegrees(Math.atan(fwd / str)) + 360)) + (MathFormulas.signumV4(str))) % 360)
        - MathFormulas.signumAngleEdition(str, fwd))
        + 360)
        % 360;

    // --------------------------------------------------------------------//
    // NAVX RESET
    // --------------------------------------------------------------------//

    if (resetNavx) {
      navx.getInstance().reset();
      Arm.getInstance().setTelescope(MKTELE.minNativePositionTelescope);
      povValue = 00;
      inverseTanAngleOG = 0;
      train.vars.avgDistTest = 0;
      train.vars.avgDistInches = 0;
      train.startDrive();
    }

    // --------------------------------------------------------------------//
    // POV ROTATION
    // --------------------------------------------------------------------//

    if (Math.abs(xbox.getRawAxis(DriveInput.rcwX)) >= 0.1 && Math.abs(xbox.getRawAxis(DriveInput.rcwY)) >= 0.1) {
      inverseTanAngleOG = ((((((Math.toDegrees(Math.atan(rcwY / rcwX)) + 360)) + (MathFormulas.signumV4(rcwX))) % 360)
          - MathFormulas.signumAngleEdition(rcwX, rcwY))
          + 360)
          % 360;

      rcw = train.moveToAngy(inverseTanAngleOG);
    }
    /*
     * if (Math.abs(rcwX) >= 0.1) {
     * navxRotate = navx.getInstance().getNavxYaw();
     * } else if (!ltrigger && isRCWrunningWithNavx) {
     * rcw = train.moveToAngy(navxRotate);
     * }
     */

    // --------------------------------------------------------------------//
    // ELSE STATEMENTS
    // --------------------------------------------------------------------//

    if (Math.abs(xbox.getRawAxis(DriveInput.rcwY)) < 0.1
        && Math.abs(xbox.getRawAxis(DriveInput.rcwX)) < 0.1) {
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

    // --------------------------------------------------------------------//
    // INTAKE
    // --------------------------------------------------------------------//
    if (rbbutton) {
      intake.rollerSet(-.7);

    } else if (lbbutton) {
      intake.rollerSet(.7);

    } else {
      intake.rollerSet(0);
    }
    if (abutton) {
      intake.toggle();
    }
    // --------------------------------------------------------------------//
    // CLAW AND ARM
    // --------------------------------------------------------------------//
    if (bbutton) {
      claw.toggle();
    }

    // --------------------------------------------------------------------//
    // OTHER
    // --------------------------------------------------------------------//
    // applying numbers

    if (xbutton) {
      april.alignToTag();
    } else if ((fwd != 0 || str != 0 || rcw != 0)) { // +,-,+
      train.etherSwerve(
          fwd / MKBABY.fwdBABY,
          str / MKBABY.strBABY,
          -rcw / MKBABY.rcwBABY,
          ControlMode.PercentOutput); // +,-,+
      /// train.setModuleDrive(ControlMode.PercentOutput, 1, 1, 1, 1);
      // train.setModuleTurn(0, 0, 0, 0);
    } else {
      train.stopEverything();
    }

    if (rtrigger && !ltrigger) {
      arm.moveArm(MathFormulas.limitAbsolute(Math.abs(xbox.getRawAxis(3)), .1),
          MathFormulas.limitAbsolute(Math.abs(xbox.getRawAxis(3)), .1));
    } else if (ltrigger && !rtrigger) {
      arm.moveArm(-MathFormulas.limitAbsolute(Math.abs(xbox.getRawAxis(2)), .1),
          -MathFormulas.limitAbsolute(Math.abs(xbox.getRawAxis(2)), .1));
    } else {
      arm.moveArm(0, 0);
    }

    if(dpaddown && !dpadup && arm.getTelescope() > MKTELE.minNativePositionTelescope)
    {
      arm.moveTele(-.23);
    }
    else if(dpadup && !dpaddown && arm.getTelescope() < MKTELE.maxNativePositionTelescope)
    {
      arm.moveTele(.23);
    }
    else 
    {
      arm.moveTele(0);
    }

    if(xbox.getRawButton(8))
    {
      arm.setTelescope(MKTELE.maxNativePositionTelescope);
    }
  }

  public void teleopDisabled() {
    resetNavx = false;
    resetDrive = false;
    xbutton = false;
    ybutton = false;
    pov = false;
    itsreal = false;
    turntesttimer.stop();
    turntesttimer.reset();
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
      train.etherSwerve(fwd, 0, rcw, ControlMode.PercentOutput);
      train.etherRCWFinder(fwd, 0, 0);
    } else {
      train.stopEverything();
    }

    SmartDashboard.putNumber("count", count);
    SmartDashboard.putNumber(
        "meastopredictratio", train.vars.avgDistInches / train.vars.avgDistTest);
    SmartDashboard.putNumber("delta", train.vars.avgDistTest);
  }

  private static class InstanceHolder {
    private static final SupaStruct mInstance = new SupaStruct();
  }
}
