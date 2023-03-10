// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class Elevator {

    private static Elevator instance;
    private DoubleSolenoid leftElevator = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
    private DoubleSolenoid rightElevator = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);
    private Timer elevatorTimer = new Timer();
    // TODO: time how long it takes for piston to move in or out, and account for that time in the isUp() and isDown() checks
    private static final double SECONDS_TO_UP = 2.1;
    private static final double SECONDS_TO_DOWN = 1.7;
    
    private static final double LIMELIGHT_UP_HEIGHT_METERS = 1.52;
    private static final double LIMELIGHT_DOWN_HEIGHT_METERS = 1.21;


    private Elevator() {}

    private enum ElevatorMode {
        elevatorUp (Value.kForward),
        elevatorDown (Value.kReverse),
        elevatorOff (Value.kOff);

        private Value value;

        private ElevatorMode(Value value) {
            this.value = value;
        }
    }
    private ElevatorMode elevatorMode = ElevatorMode.elevatorOff;
    private ElevatorMode lastElevatorMode = ElevatorMode.elevatorOff;

    private enum ElevatorState {
        playerControlled,
        autoControlled,
        autoDrivenDriverControllable;
    }
    private ElevatorState elevatorState = ElevatorState.playerControlled; //ElevatorState.autoControlled;

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }
        return instance;
    }

    public void up() {
        if (elevatorState == ElevatorState.autoDrivenDriverControllable) { elevatorState = ElevatorState.playerControlled; }
        if (elevatorState == ElevatorState.playerControlled) {
            elevatorMode = ElevatorMode.elevatorUp;
            LimelightVisionTracking.getInstance().changeHeight(LIMELIGHT_UP_HEIGHT_METERS);
        }
    }
    public void down() {
        if (elevatorState == ElevatorState.autoDrivenDriverControllable) { elevatorState = ElevatorState.playerControlled; }
        if (elevatorState == ElevatorState.playerControlled) {
            elevatorMode = ElevatorMode.elevatorDown;
            LimelightVisionTracking.getInstance().changeHeight(LIMELIGHT_DOWN_HEIGHT_METERS);
        }
    }
    public void off() {
        if (elevatorState == ElevatorState.autoDrivenDriverControllable) { elevatorState = ElevatorState.playerControlled; }
        if (elevatorState == ElevatorState.playerControlled) {
            elevatorMode = ElevatorMode.elevatorOff;
        }
    }

    public void forceUp() {
        elevatorState = ElevatorState.autoControlled;
        elevatorMode = ElevatorMode.elevatorUp;
        LimelightVisionTracking.getInstance().changeHeight(LIMELIGHT_UP_HEIGHT_METERS);
    }
    public void forceDown() {
        elevatorState = ElevatorState.autoControlled;
        elevatorMode = ElevatorMode.elevatorDown;
        LimelightVisionTracking.getInstance().changeHeight(LIMELIGHT_DOWN_HEIGHT_METERS);
    }

    public void tryDown() {
        if (elevatorState == ElevatorState.autoControlled || elevatorState == ElevatorState.autoDrivenDriverControllable) {
            elevatorState = ElevatorState.autoDrivenDriverControllable;
            elevatorMode = ElevatorMode.elevatorDown;
        }
    }

    public void enablePlayerControl() {
        elevatorState = ElevatorState.playerControlled;
    }

    public void disablePlayerControl() {
        elevatorState = ElevatorState.autoControlled;
    }

    public void update() {
        if (elevatorMode != lastElevatorMode) {
            elevatorTimer.restart();
            lastElevatorMode = elevatorMode;
        }
        leftElevator.set(elevatorMode.value);
        rightElevator.set(elevatorMode.value);
    }

    public boolean isUp() {
        return elevatorMode == ElevatorMode.elevatorUp && elevatorTimer.get() >= SECONDS_TO_UP;
    }

    public boolean isDown() {
        return elevatorMode == ElevatorMode.elevatorDown && elevatorTimer.get() >= SECONDS_TO_DOWN;
    }
}
