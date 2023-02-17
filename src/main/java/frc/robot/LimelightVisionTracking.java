package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpiutil.net.PortForwarder;

public class LimelightVisionTracking {

    private static LimelightVisionTracking instance;
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private  NetworkTableEntry tx = table.getEntry("tx");
    private NetworkTableEntry ty = table.getEntry("ty");
    private NetworkTableEntry tv = table.getEntry("tv");
    private NetworkTableEntry pipeline = table.getEntry("pipeline");
    // stream

    //fix me
    private final double REL_TARGET_HEIGHT = 98.25-16.5; // this should definitly be changed || the target height - shooter height. 
    private final double MOUNT_ANGLE = 33.5; // this should be the mount angle for the limelight + the limelight angle
    private final double DISTANCE_THRESHOLD = 200;
    private LimelightVisionTracking() {
        SmartDashboard.putNumber("limelight angle", MOUNT_ANGLE);
        // next lines litterally stollen from 5254, thanks guys for finding that darn IP and mentioning it!
        // Set up port forwarding so we can access the limelight over USB :)
        // visit '172.22.11.2:5801' in your browser (ie, Chrome) on the laptop
        // to see the limelight
        // ... maybe
        // Not sure why 5805 is required. Someone recommended it
        // Probably internal limelight networking stuff
        // PortForwarder.add(5800, "limelight.local", 5800);
        // PortForwarder.add(5801, "limelight.local", 5801);
        // PortForwarder.add(5805, "limelight.local", 5805);
    }

    public static LimelightVisionTracking getInstance() {
        if(instance == null) {
            instance = new LimelightVisionTracking();
        }
        return instance;
    }

    public double getHorizontalAngle() {
        double horizontalAngle = tx.getDouble(0.0);
        return horizontalAngle;
    } 

    public double getVerticleAngle() {
        double verticleAngle = tv.getDouble(0.0);
        return verticleAngle;
    } 

    public double getDistance() {
        if (!targetFound()) {
            return 270.0;
        }
        double angleInRadians = ((SmartDashboard.getNumber("limelight angle", MOUNT_ANGLE) + ty.getDouble(0.0)) * Math.PI) / 180;
        double distance = REL_TARGET_HEIGHT/Math.tan(angleInRadians);
        return distance;
    }

    public boolean targetFound() {
        return tv.getNumber(1).intValue() == 1;
    }

    public void optimizedDistance(){
        if(getDistance() >= DISTANCE_THRESHOLD){
            pipeline.setDouble(1.0);
        } else {
            pipeline.setDouble(0.0);
        }
    }

}