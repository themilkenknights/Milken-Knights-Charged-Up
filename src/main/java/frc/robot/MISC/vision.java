package frc.robot.MISC;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.MISC.Constants.VISION;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class vision {
    static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
static NetworkTableEntry tx = table.getEntry("tx");
static NetworkTableEntry ty = table.getEntry("ty");
static NetworkTableEntry ta = table.getEntry("ta");


private vision() {
    table.getEntry("pipeline").setValue(VISION.kLimelightPipeline);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDoubleArray(new double[6]);
  }

//read values periodically
static double x = tx.getDouble(0.0);
static double y = ty.getDouble(0.0);
static double area = ta.getDouble(0.0);

public static void UpdateVision() {
//post to smart dashboard periodically
SmartDashboard.putNumber("LimelightX", x);
SmartDashboard.putNumber("LimelightY", y);
SmartDashboard.putNumber("LimelightArea", area);
}}
