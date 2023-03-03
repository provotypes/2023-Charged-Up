package frc.robot;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


public class Arm {
    private static Arm instance;
    private WPI_TalonFX leftMotor = new WPI_TalonFX(5);
    private WPI_TalonFX rightMotor = new WPI_TalonFX(6);
    public WPI_TalonSRX sensorMotor = new WPI_TalonSRX(7);
    private Claw claw = Claw.getInstance();
    private Elevator elevator = Elevator.getInstance();


    
    
    private Arm() {
        sensorMotor.configFactoryDefault();
        leftMotor.configFactoryDefault();
        rightMotor.configFactoryDefault();
        
        sensorMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.Analog, 0, 10);
        
        leftMotor.setNeutralMode(NeutralMode.Brake);
        rightMotor.setNeutralMode(NeutralMode.Brake);
        
        rightMotor.follow(leftMotor);
        rightMotor.setInverted(TalonFXInvertType.OpposeMaster);
        
        // leftMotor.configRemoteFeedbackFilter(sensorMotor, 0);
        // leftMotor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0, 0, 10);
        
        leftMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        
        leftMotor.config_kP(0, 1); //add other PID terms if needed
        leftMotor.selectProfileSlot(0, 0);
        
        leftMotor.configOpenloopRamp(1); // change this to what seems to work
        rightMotor.configOpenloopRamp(1);
        
        leftMotor.configClosedloopRamp(2);
        // leftMotor.setSelectedSensorPosition(0); //TODO: add this somewhere else so that we can call it at the beginning of auto
        sensorMotor.setSensorPhase(true);
        
        leftMotor.setSelectedSensorPosition((ArmPosition.armInside.value * UNITS_PER_DEGREE) * GEAR_REDUCTION_MODIFIER * 4); //TODO: add this somewhere else so that we can call it at the beginning of auto

        
        
    }
    
    public static Arm getInstance() {
        if (instance == null) {
            instance = new Arm();
        }
        return instance;
    }
    
    
    public enum ArmPosition {
        armInside (-24.0), // if this position is 0, the arm's rotation is limited to only positive numbers (which makes math nicer to think about)
        armPickupFloor (7.0),
        armPickupShelf (46.0),
        armHigh (92.0),
        armLow (75.0),
        armTransport (0.0);
        
        private Double value;

        private ArmPosition(Double value) {
            this.value = value;
        }
    }

    private enum ArmState {
        driverControlled,
        autoControlled;
    }

    private ArmState armState = ArmState.driverControlled;
    private ArmPosition armPosition = ArmPosition.armInside;
    private double manualControlPower = 0;


    private final double maxGravityCompensation = .13;
    // place where the claw is forced into closed position to avoid breaking stuff
    // this value should be just outside the robot, because brian changed the claw dimensions
    //    so it doesn't fit anymore when it's open >:(
    private final double clawControlThreshold = Math.toRadians(4.5);

    // place where elevator is forced into up position so that arm fits
    private final double elevatorControlthreshold = Math.toRadians(4.5);
    // this is where the elevator can go back down, while the arm is inside the robot
    private final double elevatorStartThreshold = Math.toRadians(5.0);
    
    // speed limit for arm rotation speed
    // unit is Radians/Tick (aka Radians/ 1/20th seconds)
    private final static double armRotationRate = Math.toRadians(1.0);
    
    private final static double UNITS_PER_DEGREE = 2048.0 / 360.0; //TODO: check that it's not supposed to be 2047

    //                                                     gearbox   outside gears
    private final static double GEAR_REDUCTION_MODIFIER = (25.0/3.0);
    // -97 - shelf, 85 - high, 60 - med ,-30 - rest
    public final double shelfAngle = -97.0;
    public final double highAngle = 85.0;
    public final double mediumAngle = 60.0;
    public final double restAngle = -30.0;

    public boolean presetButtonPressed = false;
    public boolean reachedAngle = false;
    public double reachAngle = restAngle;
    public double initialAngleDelta = 0.0;

    public final double elevatorSafetyLimitMin = -21;
    public final double elevatorSafetyLimitMax = 5;

    public double elevatorLimitMin = elevatorSafetyLimitMin;
    public double elevatorLimitMax = elevatorSafetyLimitMax;


    public void clawInside() {
        armState = ArmState.autoControlled;
        armPosition = ArmPosition.armInside;
    }
    public void clawHigh() {
        armState = ArmState.autoControlled;
        armPosition = ArmPosition.armHigh;
    }
    public void clawLow() {
        armState = ArmState.autoControlled;
        armPosition = ArmPosition.armLow;
    }
    public void clawPickupShelf() {
        armState = ArmState.autoControlled;
        armPosition = ArmPosition.armPickupShelf;
    }
    public void clawPickupFloor() {
        armState = ArmState.autoControlled;
        armPosition = ArmPosition.armPickupFloor;
    }
    public void clawTransport() {
        armState = ArmState.autoControlled;
        armPosition = ArmPosition.armTransport;
    }
    public void clawManualControl(double power) {
        armState = ArmState.driverControlled;
        manualControlPower = power * .15; //TODO: adjust multiplication
        // System.out.println(manualControlPower);
    }

    public boolean isAtPosition(ArmPosition pos) {
        double angle = (leftMotor.getSelectedSensorPosition() / UNITS_PER_DEGREE) * GEAR_REDUCTION_MODIFIER;
        return (pos.value - 2.0 < angle && angle < pos.value + 2.0);
    }
    // 0.070325
    public boolean doLimiting = false;


    public double getArmAngle() {
        return (leftMotor.getSelectedSensorPosition() / UNITS_PER_DEGREE) / GEAR_REDUCTION_MODIFIER / 4;
    }

    public void update() {

        // All control of elevator, claw, and arm should happen here since the arm rotation can effect the states of the claw and elevator


        double armAngle = getArmAngle();
        System.out.println(armAngle);
        double armSin = Math.sin(Math.toRadians(armAngle));

        double targetAngle = (armState == ArmState.autoControlled) ? armPosition.value : (/*armAngle + */(1 * manualControlPower));
        // double targetAngleGearReducted = -(targetAngle / GEAR_REDUCTION_MODIFIER) * UNITS_PER_DEGREE;
        if (Math.min(elevatorLimitMin, elevatorSafetyLimitMin) <= armAngle && armAngle <= Math.max(elevatorLimitMax, elevatorSafetyLimitMax)) {
            elevator.forceUp();
            if (presetButtonPressed) {
                claw.close();
            }
        }
        else {
            elevator.enablePlayerControl();
        }


        if (doLimiting) {
            // make sure arm doesn't try to move past physical limits
            targetAngle = Math.max(Math.min(ArmPosition.armHigh.value, targetAngle), 0);

            double angleDelta = Math.max(Math.min(armRotationRate, targetAngle - armAngle), -armRotationRate);
            double estimatedAngle = armAngle + angleDelta;


            boolean canMove = true;
            boolean didMove = false;

            // check if arm is moving to a position inside the robot, above the electronics
            if (estimatedAngle < elevatorControlthreshold
            && estimatedAngle > elevatorStartThreshold) {
                // make sure the elevator is up before moving the arm the rest of the way
                if (elevator.isDown()) {
                    elevator.forceUp();
                    canMove = false;
                }
                else if (elevator.isUp()) {
                    if (canMove && !didMove) {
                        // leftMotor.set(TalonFXControlMode.Position, targetAngleGearReducted, DemandType.ArbitraryFeedForward, maxGravityCompensation * armSin);
                        didMove = true;
                    }
                }
            }

            // is arm currently in start position?
            if (armAngle < elevatorStartThreshold) {
                // check if arm is moving into the robot
                if (angleDelta <= 0) {
                    if (elevator.isUp()) {
                        elevator.tryDown();
                    }
                }
                else if (estimatedAngle >= elevatorStartThreshold) {
                    if (elevator.isDown()) {
                        elevator.forceUp();
                        canMove = false;
                    }
                    else if (elevator.isUp()) {
                        if (canMove && !didMove) {
                            // leftMotor.set(TalonFXControlMode.Position, targetAngleGearReducted, DemandType.ArbitraryFeedForward, maxGravityCompensation * armSin);
                            didMove = true;
                        }
                    }
                }
            }

            // if arm is in- and will remain in- the space outside the robot, ...
            if (armAngle >= elevatorControlthreshold
            && estimatedAngle >= elevatorControlthreshold) {
                if (elevator.isUp()) {
                    elevator.tryDown(); // only moves down if not being player-controlled
                }
                if (canMove && !didMove) {
                    // leftMotor.set(TalonFXControlMode.Position, targetAngleGearReducted, DemandType.ArbitraryFeedForward, maxGravityCompensation * armAngle);
                    didMove = true;
                }
            }

            // make sure claw is in closed position while in robot frame
            if (armAngle <= clawControlThreshold) {
                if (claw.isOpen()) {
                    claw.forceClose();
                    canMove = false;
                }
                else if (claw.isClosed()) {
                    if (canMove && !didMove) {
                        // leftMotor.set(TalonFXControlMode.Position, targetAngleGearReducted, DemandType.ArbitraryFeedForward, maxGravityCompensation * armAngle);
                        didMove = true;
                    }
                }
            }
            if (armAngle > clawControlThreshold) {
                if (claw.isClosed()) {
                    claw.tryOpen();
                }
            }

            // this makes sure the robot moves in case some edge case happens
            if (canMove && !didMove) {
                // leftMotor.set(TalonFXControlMode.Position, targetAngleGearReducted, DemandType.ArbitraryFeedForward, maxGravityCompensation * armAngle);
            }
        }
        else {

            if (presetButtonPressed) {
                if (!reachedAngle) {
                    if (initialAngleDelta >= 0) {
                        if (armAngle >= reachAngle) {
                            reachedAngle = true;
                        }
                        double armTurnSpeed = Math.max(Math.min(0.15, reachAngle - armAngle), 0.05);
                        leftMotor.set(TalonFXControlMode.PercentOutput, armTurnSpeed, DemandType.ArbitraryFeedForward, maxGravityCompensation * armSin);

                    }
                    else {
                        if (armAngle <= reachAngle) {
                            reachedAngle = true;
                        }
                        double armTurnSpeed = -Math.max(Math.min(0.15, reachAngle - armAngle), 0.05);
                        leftMotor.set(TalonFXControlMode.PercentOutput, armTurnSpeed, DemandType.ArbitraryFeedForward, maxGravityCompensation * armSin);
                    }
                }
            }
            else {
                reachedAngle = false;
                reachAngle = restAngle;
                leftMotor.set(TalonFXControlMode.PercentOutput, manualControlPower, DemandType.ArbitraryFeedForward, maxGravityCompensation * armSin);
            }
        }
        // System.out.println(maxGravityCompensation * armSin);



        // if (armState == ArmState.autoControlled) {
        //     leftMotor.set(TalonFXControlMode.Position, armPosition.value);
        // }
        // else {
        //     leftMotor.set(TalonFXControlMode.PercentOutput, manualControlPower);
        // }
        claw.update();
        elevator.update();
    }

    /**
     * Resets the angle to the starting position
     */
    public void resetAngle() {
        leftMotor.setSelectedSensorPosition((ArmPosition.armInside.value * UNITS_PER_DEGREE) * GEAR_REDUCTION_MODIFIER * 4); //TODO: add this somewhere else so that we can call it at the beginning of auto

    }

}
