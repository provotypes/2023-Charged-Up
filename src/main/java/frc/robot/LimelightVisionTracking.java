package frc.robot;


import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpiutil.net.PortForwarder;

public class LimelightVisionTracking {

    private static LimelightVisionTracking instance;
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry fieldPosition; //= table.getEntry("botpose_wpired");
    private NetworkTableEntry tx = table.getEntry("tx");
    private NetworkTableEntry ty = table.getEntry("ty");
    private NetworkTableEntry tv = table.getEntry("tv");
    private NetworkTableEntry pipeline = table.getEntry("pipeline");
    // private DriveTrain driveTrain;
    // stream

    //fix me
    private final static double MOUNT_HEIGHT = 24; //keep the units the same
    private final static double REL_TARGET_HEIGHT_TOP_TAPE = 44-MOUNT_HEIGHT; // this should definitly be changed || the target height - shooter height. 
    private final static double REL_TARGET_HEIGHT_BOTTOM_TAPE = 24-MOUNT_HEIGHT; // this should definitly be changed || the target height - shooter height. 
    private final static double MOUNT_ANGLE = 33.5; // this should be the mount angle for the limelight + the limelight angle
    //private final double DISTANCE_THRESHOLD = 200;

    public enum Targets{
        topPole (REL_TARGET_HEIGHT_TOP_TAPE),
        bottomPole (REL_TARGET_HEIGHT_BOTTOM_TAPE);

        private final double height;

        private Targets(double height) {
            this.height = height;
        }
    }

    public enum Pipelines{
        aprilTag (0),
        shinyTape (1);

        private final double id;

        private Pipelines(double id) {
            this.id = id;
        }
    }

    private LimelightVisionTracking() {
        SmartDashboard.putNumber("limelight angle", MOUNT_ANGLE);
        // next lines litterally stollen from 5254, thanks guys for finding that darn IP and mentioning it!
        // Set up port forwarding so we can access the limelight over USB :)
        // visit '172.22.11.2:5801' in your browser (ie, Chrome) on the laptop
        // to see the limelight
        // ... maybe
        // Not sure why 5805 is required. Someone recommended it
        // Probably internal limelight networking stuff
        PortForwarder.add(5800, "limelight.local", 5800);
        PortForwarder.add(5801, "limelight.local", 5801);
        PortForwarder.add(5805, "limelight.local", 5805);

        if (DriverStation.getAlliance() == Alliance.Blue) {
            fieldPosition = table.getEntry("botpose_wpiblue");
        }
        else {
            fieldPosition = table.getEntry("botpose_wpired"); //TODO add this in a periodic thing
        }
    }

    /*public void hereHaveADriveTrain(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
    }*/


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

    public double getDistance(Targets target) {
        if (!targetFound()) {
            return 270.0;
        }
        double angleInRadians = ((SmartDashboard.getNumber("limelight angle", MOUNT_ANGLE) + ty.getDouble(0.0)) * Math.PI) / 180; //maybe switch this to use `Math.toRadians()`
        double distance = target.height/Math.tan(angleInRadians);
        return distance;
    }

    public boolean targetFound() {
        return tv.getNumber(0).intValue() == 1;
    }

    /*  not sure if this is relavent anymore
    public void optimizedDistance() {
        if(getDistance() >= DISTANCE_THRESHOLD){
            pipeline.setDouble(1.0);
        } else {
            pipeline.setDouble(0.0);
        }
    }*/

    public void switchPipeline(Pipelines pipelineType) {
        pipeline.setDouble(pipelineType.id);
    }

    public void update() {
        if (targetFound() && pipeline.getInteger(0) == Pipelines.aprilTag.id) {
            double[] currentPosition = fieldPosition.getDoubleArray(new double[7]);
            DriveTrain.getInstance().addVisionPose(currentPosition);
        }
    }

    public void changeHeight(double height) { 
        table.getEntry("camera_pose_set").setDoubleArray(new double[] {.38, .1, height});
    }

}