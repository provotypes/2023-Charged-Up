// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

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

/** Add your docs here. */
public class DriveTrain {

    private DifferentialDrive driveTrain;
    private final CANSparkMax leftMotor1 = new CANSparkMax(1, MotorType.kBrushless);
    private final CANSparkMax leftMotor2 = new CANSparkMax(2, MotorType.kBrushless);
    private final CANSparkMax rightMotor1 = new CANSparkMax(3, MotorType.kBrushless);
    private final CANSparkMax rightMotor2 = new CANSparkMax(4, MotorType.kBrushless);
    private final MotorControllerGroup leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2);
    private final MotorControllerGroup rightMotors = new MotorControllerGroup(rightMotor1, rightMotor2);
    private double driveRampRate = 0.5;

    private final XboxController xboxController = new XboxController(0);
    private final Joystick joystick = new Joystick(1);
    private RelativeEncoder leftEncoder1;
    private RelativeEncoder leftEncoder2;
    private RelativeEncoder rightEncoder1;
    private RelativeEncoder rightEncoder2;
    private AHRS gyro = new AHRS();

    private double prev_speed = 0.0;
    double drive_speed = 0.0;

    private final double DISTANCE_PER_ROTATION = 1.0d / 8.0d * 6.1d * Math.PI;

    public void robotInit() {
        leftMotor1.restoreFactoryDefaults();
        leftMotor2.restoreFactoryDefaults();
        rightMotor1.restoreFactoryDefaults();
        rightMotor2.restoreFactoryDefaults();

        leftMotor1.setOpenLoopRampRate(driveRampRate);
        leftMotor2.setOpenLoopRampRate(driveRampRate);
        rightMotor1.setOpenLoopRampRate(driveRampRate);
        rightMotor2.setOpenLoopRampRate(driveRampRate);

        rightMotors.setInverted(true);
        driveTrain = new DifferentialDrive(leftMotors, rightMotors);
        driveTrain.setDeadband(0.1);

        leftEncoder1 = leftMotor1.getEncoder();
        leftEncoder2 = leftMotor2.getEncoder();
        rightEncoder1 = rightMotor1.getEncoder();
        rightEncoder2 = rightMotor2.getEncoder();

        leftEncoder1.setPositionConversionFactor(DISTANCE_PER_ROTATION);
        leftEncoder2.setPositionConversionFactor(DISTANCE_PER_ROTATION);
        rightEncoder1.setPositionConversionFactor(DISTANCE_PER_ROTATION);
        rightEncoder2.setPositionConversionFactor(DISTANCE_PER_ROTATION);

        leftMotor1.burnFlash();
        leftMotor2.burnFlash();
        rightMotor1.burnFlash();
        rightMotor2.burnFlash();
    }

    public void TeleopPer () {
        prev_speed = drive_speed;
        drive_speed = xboxController.getLeftY();

        driveTrain.arcadeDrive(drive_speed, -xboxController.getRightX() * 0.6);

    }

    public void brake() {
        leftMotor1.setIdleMode(IdleMode.kBrake);
        leftMotor2.setIdleMode(IdleMode.kCoast);
        rightMotor1.setIdleMode(IdleMode.kBrake);
        rightMotor2.setIdleMode(IdleMode.kCoast);
      }
    
      public void coast() {
        leftMotor1.setIdleMode(IdleMode.kCoast);
        leftMotor2.setIdleMode(IdleMode.kCoast);
        rightMotor1.setIdleMode(IdleMode.kCoast);
        rightMotor2.setIdleMode(IdleMode.kCoast);
      }
}
