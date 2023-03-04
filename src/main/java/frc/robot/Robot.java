// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Map;

import static java.util.Map.entry;
import java.util.concurrent.Callable;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    /*
    * This function is run when the robot is first started up and should be used for any
    * initialization code.
    */

    // instances of all other components
    public DriveTrain driveTrain = DriveTrain.getInstance();
    private SeesawAuto seesawAuto = SeesawAuto.getInstance();
    private AutoRoutine autoRoutine = AutoRoutine.getInstance();
    private Elevator elevator = Elevator.getInstance();
    private Claw claw = Claw.getInstance();
    private Arm arm = Arm.getInstance();
    private LimelightVisionTracking limelight = LimelightVisionTracking.getInstance();

    private final AnalogPotentiometer pressureSensor = new AnalogPotentiometer(0 + 4, 250, -25); //add 4 to analog in to get the port on the navX

    // alliance tracker (to avoid calling the getter method over and over)
    public static DriverStation.Alliance alliance;

    // Physical Components

    private XboxController xboxController1 = new XboxController(1);
    private XboxController xboxController2 = new XboxController(0);
    private boolean swappableControllers = false;

    // internal operation stuff
    private boolean teleopSeesawAuto = false;

    private int controllerSwapTick = 0; // idk, just in case?
    private final int controllerSwapDelay = 80;

    // dashboard stuff
    public static final ShuffleboardTab mainTab = Shuffleboard.getTab("Robot");
    private final GenericEntry pressureEntry = mainTab.add("Pnuematics Pressure", pressureSensor.get()).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", 0, "max", 120)).getEntry();


    // Control Bindings; maps a function that returns a boolean to a robot function
    private final Map<Callable<Boolean>, Runnable> controlBinds = Map.ofEntries(
        entry(() -> {return (xboxController1.getAButtonPressed());}, claw::close),
        entry(() -> {return (xboxController1.getBButtonPressed());}, claw::open),
        entry(() -> {return (xboxController1.getXButtonPressed());}, driveTrain::fullBrake),
        entry(() -> {return (xboxController1.getYButtonPressed());}, driveTrain::brake),
        //entry(() -> {return (xboxController2.getBButtonPressed());}, arm::),
        entry(() -> {return (xboxController1.getPOV() == 0 || xboxController2.getPOV() == 0);}, elevator::up),
        entry(() -> {return (/*xboxController1.getPOV() == 180 ||*/ xboxController2.getPOV() == 180);}, elevator::down),
        entry(() -> {return (xboxController2.getLeftTriggerAxis() >= 0.75);}, claw::open),
        entry(() -> {return (xboxController2.getRightTriggerAxis() >= 0.75);}, claw::close),
        entry(() -> {return xboxController1.getRightBumperPressed();}, () -> {teleopSeesawAuto = true;driveTrain.fullBrake();}),
        entry(() -> {return xboxController1.getLeftBumperPressed();}, () -> {teleopSeesawAuto = false;driveTrain.brake();})
    );


    @Override
    public void robotInit() {
        alliance = DriverStation.getAlliance();

    }

    @Override
    public void robotPeriodic() {
        driveTrain.updatePose();
        limelight.update();

        pressureEntry.setDouble(pressureSensor.get());

        // System.out.println(arm.sensorMotor.getSelectedSensorPosition());
    }

    @Override
    public void autonomousInit() {
        /* autoRoutine.dockingPosition = null; // get from shuffleboard or something
        autoRoutine.routine = null; // also this ^ */ 
        // now found in the AutoRoutine class

        arm.resetAngle();
        autoRoutine.initAuto();
    }

    @Override
    public void autonomousPeriodic() {
        autoRoutine.doRoutine();
    }

    @Override
    public void teleopInit() {
        driveTrain.brake();
    }

    @Override
    public void teleopPeriodic() {

        driveTrain.arcadeDrive(xboxController1.getLeftY() * 0.7/0.6, xboxController1.getRightX()); //left Y is negative normally, so we flip it
        arm.clawManualControl(-xboxController2.getRightY());

        controlBinds.forEach((Callable<Boolean> condition, Runnable event) -> {
            try {
                if (condition.call()) {
                    teleopSeesawAuto = false; // do this before event.run(), because thats where teleopSeesawAuto may be set to true so... yeah
                    event.run();
                }
            } catch (Exception e) {
                e.printStackTrace();
                // idk if anything can/should be done here
            }
        });

        if (swappableControllers && xboxController1.getRightTriggerAxis() >= 0.95 && xboxController2.getRightTriggerAxis() >= 0.95) {
            controllerSwapTick += 1;
            if (controllerSwapTick >= controllerSwapDelay) {
                controllerSwapTick = 0;
                XboxController temp = xboxController1;
                xboxController1 = xboxController2;
                xboxController2 = temp;
            }
        } else if (swappableControllers) {
            controllerSwapTick = 0;
        };


        if (xboxController2.getAButton()) {
            if (xboxController2.getAButtonPressed()) {
                arm.reachAngle = arm.restAngle;
                arm.presetButtonPressed = true;
                arm.initialAngleDelta = arm.reachAngle - arm.getArmAngle();
                arm.elevatorLimitMax = 15;
                arm.elevatorLimitMin = -31;
            }
        }
        else if (xboxController2.getBButton()) {
            if (xboxController2.getBButtonPressed()) {
                arm.reachAngle = arm.shelfAngle;
                arm.presetButtonPressed = true;
                arm.initialAngleDelta = arm.reachAngle - arm.getArmAngle();
                arm.elevatorLimitMax = 15;
                arm.elevatorLimitMin = -31;
            }
        }
        else if (xboxController2.getXButton()) {
            if (xboxController2.getXButtonPressed()) {
                arm.reachAngle = arm.mediumAngle;
                arm.presetButtonPressed = true;
                arm.initialAngleDelta = arm.reachAngle - arm.getArmAngle();
                arm.elevatorLimitMax = 15;
                arm.elevatorLimitMin = -31;
            }
        }
        else if (xboxController2.getYButton()) {
            if (xboxController2.getYButtonPressed()) {
                arm.reachAngle = arm.highAngle;
                arm.presetButtonPressed = true;
                arm.initialAngleDelta = arm.reachAngle - arm.getArmAngle();
                arm.elevatorLimitMax = 15;
                arm.elevatorLimitMin = -31;
            }
        }
        else {
            arm.reachAngle = arm.restAngle;
            arm.presetButtonPressed = false;
            arm.elevatorLimitMax = arm.elevatorSafetyLimitMax;
            arm.elevatorLimitMin = arm.elevatorSafetyLimitMin;
        }


        // if (xboxController.getAButtonPressed()) {
        //     elevator.up();
        // }
        // else if (xboxController.getBButtonPressed()) {
        //     elevator.down();
        // }

        if (teleopSeesawAuto) {
            seesawAuto.autoPark();
        }

        arm.update(); // arm calls claw and elevator updates
        // claw.update();
        // elevator.update();
    }

    @Override
    public void disabledInit() {
        driveTrain.fullBrake();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void testInit() {
        arm.resetAngle();
        driveTrain.coast();
        // arm.setupRightArm();
    }

    @Override
    public void testPeriodic() {
        
    }

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}

}
