package frc.robot.MISC;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.MISC.Constants.VISION;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private static NetworkTableEntry tx = table.getEntry("tx");
    private static NetworkTableEntry ty = table.getEntry("ty");
    private static NetworkTableEntry ta = table.getEntry("ta");

    // Global static variables for x, y, and area.
    public static double x = 0.0;
    public static double y = 0.0;
    public static double area = 0.0;

    // Since this class seems to be utility-like and not meant to be instantiated, 
    // the constructor is made private to ensure this.
    private Vision() {
    }

    public static void init() {
        updateVision();
    }

    public static void updateVision() {
        // Read values from NetworkTable
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);

        // Post to SmartDashboard
        postToDashboard();
    }

    public static void postToDashboard() {
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
    }
}
