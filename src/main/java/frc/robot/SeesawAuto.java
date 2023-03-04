// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// on rick rover positive pitch is left, positive roll is foreward, positive yaw is clockwise

public class SeesawAuto {

    private static SeesawAuto instance;
    private DriveTrain driveTrain = DriveTrain.getInstance();

    private SeesawAuto() {}

    public static SeesawAuto getInstance() {
        if (instance == null) {
            instance = new SeesawAuto();
        }
        return instance;
    }

    // Measured in degrees
    // this is how close to 0|180 the robot has to get before forward/backward movement can happen
    private final double yawPercision = 5.0;
    private final double pitchPercision = 1.0;
    private final double maxTurnRate = 0.5;
    private final double maxDriveSpeed = 1;
    private final double rampAngle = 7.0;
    private double startSpeed = 0.8;
    private double speedMod = 0.0;
    private int switches = 1;
    private int tiltSide = 1;


    private final double pitchModifier = 1.0;

    public void autoPark() {
        float pitch = driveTrain.gyro.getRoll();

        // float roll = driveTrain.gyro.getRoll();
        // float yaw = driveTrain.gyro.getYaw();
        double yaw = driveTrain.getRotation(); // clockwise positive, returns between 0.0 (inclusive) and 360.0 (exclusive)
        
        while (yaw >= 360) {yaw -= 360;}
        while (yaw < 0) {yaw += 360;}

        System.out.println("pitch: " + (int) pitch + "  yaw: " + (int) yaw);

        double turnRate = 0.0; // remember: for arcadeDrive(), counterclockwise is positive

        if ((0 < yaw && yaw < 90) ||
        (180 < yaw && yaw < 270)) {
            // turn left (turnRate is set to a positive number)
            turnRate = Math.max(Math.min(maxTurnRate, -((yaw > 180.0) ? (yaw - 180.0) : yaw) / 2.0), -maxTurnRate);
        }
        if ((90 < yaw && yaw < 180)
        || (270 < yaw && yaw < 360)) {
            // turn right (turnRate is set to a negative number)
            turnRate = Math.max(Math.min(maxTurnRate, ((yaw > 180.0) ? (yaw - 180.0) : yaw) / 2.0), -maxTurnRate);
        }

        // set drive speed to non-zero value if:
        //   1: pitch is significantly different from 0
        //   2: yaw is close to 0 or 180
        double driveSpeed;
        
        if (-pitch > rampAngle) {
            // if (tiltSide == 1) {
            //     tiltSide = 2;
            //     switches = Math.max(5, switches + 1);
            // }
            
            driveSpeed = (startSpeed - (speedMod * switches));
        }
        else if (pitch > rampAngle) {
            // if (tiltSide == 2) {
            //     tiltSide = 1;
            //     switches = Math.max(5, switches + 1);
            // }
            driveSpeed = -(startSpeed - (speedMod * switches));
        }
        else {
            driveSpeed = 0;
        }
        // if (!(startSpeed - (speedMod * switches) < 0)) {
        // }
        // else {
        //     driveSpeed = 0;
        // }

        
        driveTrain.arcadeDrive(driveSpeed, turnRate);
    }
}
