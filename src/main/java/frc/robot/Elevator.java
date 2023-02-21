// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/** Add your docs here. */
public class Elevator {

    private static Elevator instance;
    private DoubleSolenoid leftElevator = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    private DoubleSolenoid rightElevator = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 4);

    private Elevator() {
    }

    private enum ElevatorModes {
        elevatorUp (Value.kForward),
        elevatorDown (Value.kReverse),
        elevatorOff (Value.kOff);

        private Value value;

        private ElevatorModes(Value value) {
            this.value = value;
        }
    }
    private ElevatorModes state = ElevatorModes.elevatorOff;

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }
        return instance;
    }

    public void out() {
        state = ElevatorModes.elevatorUp;
    }
    public void in() {
        state = ElevatorModes.elevatorDown;
    }
    public void off() {
        state = ElevatorModes.elevatorOff;
    }

    public void update() {
        leftElevator.set(state.value);
        rightElevator.set(state.value);
    }

    public boolean isUp() {
        if (state == ElevatorModes.elevatorUp) {
            return true;
        }
        return false;
    }

    public boolean isDown() {
        if (state == ElevatorModes.elevatorDown) {
            return true;
        }
        return false;
    }
}
