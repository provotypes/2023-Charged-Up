// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/** Add your docs here. */
public class DriveTrain {

    private static DriveTrain instance;

    private DifferentialDrive differentialDrive;
    private final CANSparkMax leftMotor1 = new CANSparkMax(1, MotorType.kBrushless);
    private final CANSparkMax leftMotor2 = new CANSparkMax(2, MotorType.kBrushless);
    private final CANSparkMax rightMotor1 = new CANSparkMax(3, MotorType.kBrushless);
    private final CANSparkMax rightMotor2 = new CANSparkMax(4, MotorType.kBrushless);
    private final MotorControllerGroup leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2);
    private final MotorControllerGroup rightMotors = new MotorControllerGroup(rightMotor1, rightMotor2);
    private final double driveRampRate = 0.5;

    private DifferentialDriveKinematics kinematics;
    private DifferentialDriveOdometry odometry;
    private Field2d field = new Field2d();

    private RelativeEncoder leftEncoder1;
    private RelativeEncoder leftEncoder2;
    private RelativeEncoder rightEncoder1;
    private RelativeEncoder rightEncoder2;
    public AHRS gyro = new AHRS();

    private DifferentialDrivetrainSim driveSim;

    private double prev_speed = 0.0;
    double drive_speed = 0.0;

    

    // Distance per rotation: (1/8 = gear reduction) * diameter of wheel * pi
    private final double DISTANCE_PER_ROTATION = 1.0d / 8.0d * 6.1d * Math.PI; // TODO: check if this is true with new bot

    private DriveTrain() {
        leftMotor1.restoreFactoryDefaults();
        leftMotor2.restoreFactoryDefaults();
        rightMotor1.restoreFactoryDefaults();
        rightMotor2.restoreFactoryDefaults();

        leftMotor1.setOpenLoopRampRate(driveRampRate);
        leftMotor2.setOpenLoopRampRate(driveRampRate);
        rightMotor1.setOpenLoopRampRate(driveRampRate);
        rightMotor2.setOpenLoopRampRate(driveRampRate);

        rightMotors.setInverted(true);
        differentialDrive = new DifferentialDrive(leftMotors, rightMotors);
        differentialDrive.setDeadband(0.1);

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

        kinematics =
            new DifferentialDriveKinematics(Units.inchesToMeters(21.63));
        odometry = 
            new DifferentialDriveOdometry(gyro.getRotation2d(), leftEncoder1.getPosition(), rightEncoder2.getPosition()); //TODO check if units are in meters

        if (RobotBase.isSimulation()) {
            driveSim = new DifferentialDrivetrainSim( //TODO all the values below are from example code and need to be checked
                DCMotor.getNEO(2),       // 2 NEO motors on each side of the drivetrain.
                7.29,                    // 7.29:1 gearing reduction.
                7.5,                     // MOI of 7.5 kg m^2 (from CAD model).
                60.0,                    // The mass of the robot is 60 kg.
                Units.inchesToMeters(3), // The robot uses 3" radius wheels.
                Units.inchesToMeters(21.63),                  // The track width is 0.7112 meters.
                null);

        }

        SmartDashboard.putData(field);
        SmartDashboard.putData(differentialDrive);
    }

    public static DriveTrain getInstance() {
        if (instance == null) {
            instance = new DriveTrain();
        }
        return instance;
    }

    public void arcadeDrive(double forwardSpeed, double turning) {
        prev_speed = drive_speed;
        drive_speed = forwardSpeed;

        differentialDrive.arcadeDrive(drive_speed, turning * 0.6);

    }

    public void updatePose() {
        if (RobotBase.isSimulation()) {
            driveSim.setInputs(leftMotors.get() * RobotController.getInputVoltage(),
                rightMotors.get() * RobotController.getInputVoltage());
            driveSim.update(0.02);
            leftEncoder1.setPosition(driveSim.getLeftPositionMeters()/DISTANCE_PER_ROTATION);
            rightEncoder1.setPosition(driveSim.getRightPositionMeters()/DISTANCE_PER_ROTATION);

            int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
            SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
            angle.set(driveSim.getHeading().getDegrees()); //figure out if sim heading needs to be negative or not
        }

        odometry.update(gyro.getRotation2d(), leftEncoder1.getPosition(), rightEncoder1.getPosition());
        field.setRobotPose(odometry.getPoseMeters());
    }

    public void resetPose(Pose2d newPose) {
        odometry.resetPosition(gyro.getRotation2d(), leftEncoder1.getPosition(), rightEncoder1.getPosition(), newPose);
    }

    public void resetPose(double[] llData) {
        Translation2d tran2d = new Translation2d(llData[0], llData[1]);
        Rotation2d r2d = new Rotation2d(Units.degreesToRadians(llData[5]));
        this.resetPose(new Pose2d(tran2d, r2d));
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
