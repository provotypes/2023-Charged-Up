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
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// on rick rover positive pitch is left, positive roll is foreward, positive yaw is clockwise

/** Add your docs here. */
public class SeesawAuto {

    public static DriveTrain thisDriveTrain = Robot.driveTrain;
    public static AHRS gyro = thisDriveTrain.gyro;
    
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
