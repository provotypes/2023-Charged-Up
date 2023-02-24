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
    
    public void autoPark(){
        float pitch = driveTrain.gyro.getPitch();
        float roll = driveTrain.gyro.getRoll();
        float yaw = driveTrain.gyro.getYaw();

        // float threshold = 5; // this might be useful, I just don't need it rn
        
        System.out.println(roll);

        /* Math Stuffs
         * 
         * Target Values to balance: (assuming pitch is front/back tilt and roll is left/right tilt)
         * yaw ~= 0 or 180
         * pitch ~= 0
         * roll ~= 0
         * 
         * psudo code:
         * 
         * if pitch > 0 and roll ~= 0:
         *  drive backwards, speed based on pitch
         * 
         * if pitch < 0 and roll ~= 0:
         *  drive forwards, speed based on pitch
         * 
         * if roll > 0:
         *  if pitch > 0:
         *   turn right
         *  if pitch < 0:
         *   turn left
         * 
         * if roll < 0:
         *  if pitch > 0:
         *   turn left
         *  if pitch < 0:
         *   turn right
         * 
         * if 0 < yaw < 90 or 180 < yaw < 270:
         *  turn left
         * 
         * if -90 < yaw < 0 or 90 < yaw < 180:
         *  turn right
         * 
         */
        
        
    }
}
