// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;

import com.kauailabs.navx.frc.AHRS;

// on rick rover positive pitch is left, positive roll is foreward, positive yaw is clockwise

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
