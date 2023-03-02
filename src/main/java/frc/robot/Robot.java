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

    private final AnalogPotentiometer pressureSensor = new AnalogPotentiometer(0, 250, -25);

    // alliance tracker (to avoid calling the getter method over and over)
    public static DriverStation.Alliance alliance;

    // Physical Components
    private final XboxController xboxController1 = new XboxController(0);
    private final XboxController xboxController2 = new XboxController(1);

    // internal operation stuff
    private boolean teleopSeesawAuto = false;

    private int controllerSwapTick = 0; // idk, just in case?
    private final int controllerSwapDelay = 80;

    // dashboard stuff
    public static final ShuffleboardTab mainTab = Shuffleboard.getTab("Robot");
    private final GenericEntry pressureEntry = mainTab.add("Pnuematics Pressure", pressureSensor.get()).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", 0, "max", 120)).getEntry();


    // Control Bindings; maps a function that returns a boolean to a robot function
    private Map<Callable<Boolean>, Runnable> controlBinds = Map.ofEntries(
        entry(() -> {return xboxController1.getAButtonPressed();}, claw::close),
        entry(() -> {return xboxController1.getBButtonPressed();}, claw::open),
        entry(() -> {return (xboxController1.getPOV() == 0);}, elevator::up),
        entry(() -> {return (xboxController1.getPOV() == 180);}, elevator::down),
        entry(() -> {return (xboxController2.getPOV() == 0);}, arm::clawHigh),
        entry(() -> {return (xboxController2.getPOV() == 180);}, arm::clawLow),
        entry(() -> {return (xboxController2.getPOV() == 90);}, arm::clawInside),
        entry(() -> {return (xboxController2.getPOV() == 270);}, arm::clawTransport),
        entry(() -> {return xboxController2.getAButtonPressed();}, arm::clawPickupFloor),
        entry(() -> {return xboxController2.getBButtonPressed();}, arm::clawPickupShelf),
        entry(() -> {return xboxController1.getRightBumperPressed();}, () -> {teleopSeesawAuto = true;}),
        entry(() -> {return xboxController1.getLeftBumperPressed();}, () -> {teleopSeesawAuto = false;})//,
        //entry(() -> {if (xboxController1.) {}; return false;}, () -> {})
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
    }

    @Override
    public void autonomousInit() {
        /* autoRoutine.dockingPosition = null; // get from shuffleboard or something
        autoRoutine.routine = null; // also this ^ */ 
        // now found in the AutoRoutine class
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

        driveTrain.arcadeDrive(-xboxController1.getLeftY(), xboxController1.getRightX()); //left Y is negative normally, so we flip it
        
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
        //claw.update();
        //elevator.update();
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
