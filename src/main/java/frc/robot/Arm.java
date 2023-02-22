package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


public class Arm {
    private static Arm instance;
    private WPI_TalonFX leftMotor = new WPI_TalonFX(0);
    private WPI_TalonFX rightMotor = new WPI_TalonFX(1);
    private WPI_TalonSRX sensorMotor = new WPI_TalonSRX(7);
    private Claw claw = Claw.getInstance();
    private Elevator elevator = Elevator.getInstance();


    private Arm() {
        sensorMotor.configFactoryDefault();
        leftMotor.configFactoryDefault();
        rightMotor.configFactoryDefault();

        sensorMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.PulseWidthEncodedPosition, 0, 10);
        
        leftMotor.setNeutralMode(NeutralMode.Brake);
        rightMotor.setNeutralMode(NeutralMode.Brake);

        rightMotor.follow(leftMotor);
        rightMotor.setInverted(TalonFXInvertType.OpposeMaster);

        leftMotor.configRemoteFeedbackFilter(sensorMotor, 0);
        leftMotor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0, 0, 10);

        leftMotor.config_kP(0, 1); //add other PID terms if needed
        leftMotor.selectProfileSlot(0, 0);

        leftMotor.configOpenloopRamp(1); // change this to what seems to work
        rightMotor.configOpenloopRamp(1);
    }

    public static Arm getInstance() {
        if (instance == null) {
            instance = new Arm();
        }
        return instance;
    }

    private enum ArmPosition {
        armInside (Math.toRadians(0.0)), // if this position is 0, the arm's rotation is limited to only positive numbers (which makes math nicer to think about)
        armPickupFloor (Math.toRadians(40.0)),
        armPickupShelf (Math.toRadians(0.0)),
        armHighShelf (Math.toRadians(0.0)),
        armLowShelf (Math.toRadians(0.0)),
        armLowPost (Math.toRadians(0.0)),
        armHighPost (Math.toRadians(0.0));

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


    // place where the claw is forced into open position to avoid breaking stuff
    // this value should be just barely further into the robot then when the arm is straight up and down
    //   this is so that the claw can go inside the frame and keep hold of a game piece
    private final double clawControlThreshold = Math.toRadians(25.0);
    
    
    // place where elevator is forced into up position so that arm fits
    private final double elevatorControlthreshold = Math.toRadians(37.5);
    
    // this is where the elevator can go back down, while the arm is inside the robot
    private final double elevatorStartThreshold = Math.toRadians(5.0);
    
    // speed limit for arm rotation speed
    // unit is Radians/Tick (aka Radians/ 1/20th seconds)
    private final double armRotationRate = Math.toRadians(1.0);


    public void clawInside() {
        armState = ArmState.autoControlled;
        armPosition = ArmPosition.armInside;
    }
    public void clawHighPost() {
        armState = ArmState.autoControlled;
        armPosition = ArmPosition.armHighPost;
    }
    public void clawLowPost() {
        armState = ArmState.autoControlled;
        armPosition = ArmPosition.armLowPost;
    }
    public void clawHighShelf() {
        armState = ArmState.autoControlled;
        armPosition = ArmPosition.armHighShelf;
    }
    public void clawLowShelf() {
        armState = ArmState.autoControlled;
        armPosition = ArmPosition.armLowShelf;
    }
    public void clawPickupShelf() {
        armState = ArmState.autoControlled;
        armPosition = ArmPosition.armPickupShelf;
    }
    public void clawPickupFloor() {
        armState = ArmState.autoControlled;
        armPosition = ArmPosition.armPickupFloor;
    }
    public void clawManualControl(double power) {
        manualControlPower = power;
    }


    public void update() {


        // All control of elevator, claw, and arm should happen here since the arm rotation can effect the states of the claw and elevator

        double armAngle = 0.0d; // TODO: Encoder magic. this is the current angle of the arm
        double targetAngle = 0.0d; // where the arm wants to end up. (either an armPosition enum value, or calculated based on user input)


        double angleDelta = Math.max(Math.min(armRotationRate, targetAngle - armAngle), -armRotationRate);
        double estimatedAngle = armAngle + angleDelta;

        // check if arm is moving to a position inside the robot, above the electronics
        if (estimatedAngle < elevatorControlthreshold
        && estimatedAngle > elevatorStartThreshold) {
            // make sure the elevator is up before moving the arm the rest of the way
            if (elevator.isDown()) {
                elevator.forceUp();
            }
            else if (elevator.isUp()) {
                // move arm to targetAngle, bound between elevatorStartThreshold and elevatorControlThreshold
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
                }
                else if (elevator.isUp()) {
                    // move arm to targetAngle, don't go past elevatorControlThreshold though
                }
            }
        }

        // if arm is in- and will remain in- the space outside the robot, ...
        if (armAngle >= elevatorControlthreshold
        && estimatedAngle >= elevatorControlthreshold) {
            if (elevator.isUp()) {
                elevator.tryDown(); // only moves down if not being player-controlled
            }

            // move arm to targetAngle

        }

        if (armAngle <= clawControlThreshold) {
            if (claw.isClosed()) {
                claw.open();
            }
        }


        // if (armState == ArmState.autoControlled) {
        //     leftMotor.set(TalonFXControlMode.Position, armPosition.value);
        // }
        // else {
        //     leftMotor.set(TalonFXControlMode.PercentOutput, manualControlPower);
        // }f
        claw.update();
        elevator.update();
    }

}
