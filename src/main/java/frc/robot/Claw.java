// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/** Add your docs here. */
public class Claw {

    private static Claw instance;
    private DoubleSolenoid leftClawArm = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
    //private DoubleSolenoid rightClawArm = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);

    private Claw() {
    }

    private enum ClawModes {
        clawClosed (Value.kForward),
        clawOpen (Value.kReverse),
        clawOff (Value.kOff);

        private Value value;

        private ClawModes(Value value) {
            this.value = value;
        }
    }
    private ClawModes clawMode = ClawModes.clawOff;

    private enum ClawState {
        playerControlled,
        autoControlledPlayerControllable,
        autoControlled;
    }
    private ClawState clawState = ClawState.playerControlled;


    public static Claw getInstance() {
        if (instance == null) {
            instance = new Claw();
        }
        return instance;
    }

    public void open() {
        if (clawState == ClawState.autoControlledPlayerControllable) {clawState = ClawState.playerControlled;}
        if (clawState == ClawState.playerControlled) {
            clawMode = ClawModes.clawOpen;
        }
    }
    public void close() {if (clawState == ClawState.autoControlledPlayerControllable) {clawState = ClawState.playerControlled;}
        if (clawState == ClawState.playerControlled) {
            clawMode = ClawModes.clawClosed;
        }
    }
    public void off() {
        if (clawState == ClawState.autoControlledPlayerControllable) {clawState = ClawState.playerControlled;}
        if (clawState == ClawState.playerControlled) {
            clawMode = ClawModes.clawOff;
        }
    }
    public void forceClose() {
        clawMode = ClawModes.clawClosed;
        clawState = ClawState.autoControlled;
    }
    public void forceOpen() {
        clawMode = ClawModes.clawOpen;
        clawState = ClawState.autoControlled;
    }
    public void tryOpen() {
        clawMode = ClawModes.clawOpen;
        clawState = ClawState.autoControlledPlayerControllable;
    }
    public void tryClose() {
        clawMode = ClawModes.clawClosed;
        clawState = ClawState.autoControlledPlayerControllable;
    }

    public void update() {
        leftClawArm.set(clawMode.value);
        //rightClawArm.set(clawMode.value);
    }

    public boolean isOpen() {
        return clawMode == ClawModes.clawOpen;
    }

    public boolean isClosed() {
        return clawMode == ClawModes.clawClosed;
    }
}
