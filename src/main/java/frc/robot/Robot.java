// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;

import com.kauailabs.navx.frc.AHRS;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  public DriveTrain driveTrain = DriveTrain.getInstance();
  private SeesawAuto seesawAuto = SeesawAuto.getInstance();
  private Elevator elevator = Elevator.getInstance();

  private final XboxController xboxController = new XboxController(0);
  private final Joystick joystick = new Joystick(1);

  @Override
  public void robotInit() {
  }

  @Override
  public void robotPeriodic() {
    driveTrain.updatePose();

    /*if (limelight sees AprilTag) {
      driveTrain.resetPose(thing);
    } */
  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {
    seesawAuto.autoPark();

  }

  @Override
  public void teleopInit() {
    driveTrain.brake();
  }

  @Override
  public void teleopPeriodic() {
    driveTrain.arcadeDrive(-xboxController.getLeftY(), xboxController.getRightX()); //left Y is negative normally, so we flip it

    if (xboxController.getAButtonPressed()) {
      elevator.up();
    }
    else if (xboxController.getAButtonReleased()) {
      elevator.down();
    }

    elevator.update();
  }

  @Override
  public void disabledInit() {
    driveTrain.coast();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}


}
