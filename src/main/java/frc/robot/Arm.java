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
    }

    public static Arm getInstance() {
        if (instance == null) {
            instance = new Arm();
        }
        return instance;
    }

    private enum ArmPosition {
        armInside (0.0),
        armPickupShelf (0.5),
        armHighShelf (0.5),
        armLowShelf (0.5),
        armLowPost (0.5),
        armHighPost (0.1);

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
    public void clawManualControl(double power) {
        manualControlPower = power;
    }


    public void update() {
        /*if (armState == ArmState.autoControlled) {

        }
        else if (armState == ArmState.driverControlled) {

        }*/
        if (armState == ArmState.autoControlled) {
            leftMotor.set(TalonFXControlMode.Position, armPosition.value);
        }
        else {
            leftMotor.set(TalonFXControlMode.PercentOutput, manualControlPower);
        }
    }

}
