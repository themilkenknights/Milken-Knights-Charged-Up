package frc.robot.MECHANISMS.ARM;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.MISC.Constants.CANID;
import frc.robot.MISC.Constants.MKARM;
import frc.robot.MISC.Constants.MKTELE;
import frc.robot.MISC.Constants.MKWRIST;
import frc.robot.MISC.MathFormulas;
import frc.robot.MISC.Motor;
import com.revrobotics.SparkMaxRelativeEncoder;

public class wrist {


  private Motor motor = Motor.getInstance();
private PIDController wristpid;
private CANSparkMax wristmotor;
private RelativeEncoder m_encoder;
private SparkMaxPIDController m_pidController;

  private wrist() {
    wristpid = new PIDController(MKWRIST.kP, MKWRIST.kI, MKWRIST.kD);
    wristmotor = motor.Sparky(CANID.wristCANID);
    m_encoder = wristmotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);
    /**
     * In order to use PID functionality for a controller, a SparkMaxPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_pidController = wristmotor.getPIDController();
  
    /**
     * The PID Controller can be configured to use the analog sensor as its feedback
     * device with the method SetFeedbackDevice() and passing the PID Controller
     * the CANAnalog object. 
     */
    m_pidController.setFeedbackDevice(m_encoder);

    m_pidController.setP(MKWRIST.kP);
    m_pidController.setI(MKWRIST.kI);
    m_pidController.setD(MKWRIST.kD);
    m_pidController.setIZone(0);
    m_pidController.setFF(0);
    m_pidController.setOutputRange(-1, 1);
  }

  public static wrist getInstance() {
    return InstanceHolder.mInstance;
  }

  public void movewrist(double setpoint) {
    wristmotor.set(setpoint);
  }

  public double getwrist()
  {
    return m_encoder.getPosition();
  }

  public void setWristPID(double setpoint)
  {
    m_pidController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
  }

  /**
   * This function assumes that the base of the arm is at the origin (0,0) and the angles are
   * measured from the horizontal line. To account for gravity, you can add a third link that is
   * perpendicular to the base of the arm and always points downward. You can then use the same
   * methods to calculate the angle for this third link. openai
   *
   * @param link1
   * @param link2
   * @param x
   * @param y
   */
  public static void getAngles(double link1, double link2, double x, double y) {
    // Calculate the distance from the base of the arm to the target position
    double dist = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

    // Use the Law of Cosines to find the angle between link1 and the horizontal
    // line
    double link1Angle =
        Math.acos(
            (Math.pow(link1, 2) + Math.pow(dist, 2) - Math.pow(link2, 2)) / (2 * link1 * dist));

    // Use the Law of Sines to find the angle between link2 and the horizontal line
    double link2Angle = Math.asin((link2 * Math.sin(link1Angle)) / dist);

    // Return the angles in degrees
    System.out.println("Link 1 angle: " + Math.toDegrees(link1Angle));
    System.out.println("Link 2 angle: " + Math.toDegrees(link2Angle));
  }

  /**
   * 3 joint arm https://www.chiefdelphi.com/t/pid-tuning-for-3-joint-arm/347116/15
   *
   * @param ang1
   * @param ang2
   * @param ang3
   * @param lengths
   * @return
   */
  public static double[] getXY(double ang1, double ang2, double ang3, double[] lengths) {
    double x = getX(ang1, ang2, ang3, lengths);
    double y = getY(ang1, ang2, ang3, lengths);

    double[] xy = {x, y};
    return xy;
  }

  public static double getX(double ang1, double ang2, double ang3, double[] lengths) {
    double a = Math.abs(ang1 + ang2 - 180);
    double b = Math.abs(ang1 + ang2 + ang3 - 360);

    double realAng1 = 0;
    if (ang1 > 90) // if l1 is pointed backwards
    realAng1 = 180 - ang1;
    else realAng1 = ang1;

    double x1 = lengths[0] * Math.cos(realAng1);
    double x2 = lengths[1] * Math.cos(a);
    double x3 = lengths[2] * Math.cos(b);

    double len = 0;

    if (ang1 > 90) // if l1 is pointed backwards
    len -= x1;
    else len += x1;
    len += x2;
    len += x3;

    return len;
  }

  public static double getY(double ang1, double ang2, double ang3, double[] lengths) {
    double a = Math.abs(ang1 + ang2 - 180);
    double b = Math.abs(ang1 + ang2 + ang3 - 360);

    double realAng1 = 0;
    if (ang1 > 90) // if l1 is pointed backwards
    realAng1 = 180 - ang1;
    else realAng1 = ang1;

    double y1 = lengths[0] * Math.sin(realAng1);
    double y2 = lengths[1] * Math.sin(a);
    double y3 = lengths[2] * Math.sin(b);

    double len = y1;

    if (ang1 > 90) // if l1 is tilted backward
    {
      if (90 - realAng1 + ang2 > 90) // if l2 is tilted up
      {
        len += y2;
        if (a + ang3 > 180) // if l3 is tilted up
        len += y3;
        else // if l3 is tilted down
        len -= y3;
      } else // if l2 is tilted down
      {
        len -= y2;
        if (a + 180 < ang3) // if l3 is tilted up
        len += y3;
        else // if l3 is tilted down
        len -= y3;
      }
    } else // if l1 is tilted forward
    {
      if (ang1 + ang2 > 180) // if l2 is tilted up
      {
        len += y2;
        if (a + ang3 > 180) // if l3 is tilted up
        len += y3;
        else // if l3 is tilted down
        len -= y3;
      } else // if l2 is tilted down
      {
        len -= y2;
        if (a + 180 < ang3) // if l3 is tilted up
        len += y3;
        else // if l3 is tilted down
        len -= y3;
      }
    }

    return len;
  }

  // https://www.chiefdelphi.com/t/velocity-limiting-pid/164908/22
  // feed forward for arm

  private static class InstanceHolder {
    private static final wrist mInstance = new wrist();
  }
}
