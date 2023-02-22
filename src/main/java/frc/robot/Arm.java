package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


public class Arm {
    private static Arm instance;
    private WPI_TalonFX leftMotor = new WPI_TalonFX(0);
    private WPI_TalonFX rightMotor = new WPI_TalonFX(1);

    private Arm() {}

    public Arm getInstance() {
        if (instance == null) {
            instance = new Arm();
        }
        return instance;
    }

    private enum ArmPosition {
        armInside (0.5),
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


    public void update() {
        if (armState == ArmState.autoControlled) {

        }
        else if (armState == ArmState.driverControlled) {

        }
    }

}
