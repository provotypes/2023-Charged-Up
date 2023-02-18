// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.kauailabs.navx.frc.AHRS;

// on rick rover positive pitch is left, positive roll is foreward, positive yaw is clockwise

/** Add your docs here. */
public class SeesawAuto {
    private DriveTrain thisDriveTrain;
    private AHRS gyro;

    public SeesawAuto(DriveTrain thisDriveTrain, AHRS gyro) {
        this.thisDriveTrain = thisDriveTrain;
        this.gyro = gyro;
    }

    
    public void autoPark(){
        float pitch = gyro.getPitch();
        float roll = gyro.getRoll();
        float yaw = gyro.getYaw();

        float threshold = 5;
        
        System.out.println(roll);

        
        if (pitch < -threshold){
            //correct urself
        }
        else if (pitch > threshold){
            //correct urself
        }

        
    }
}
