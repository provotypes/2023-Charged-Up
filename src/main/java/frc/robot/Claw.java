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
    private DoubleSolenoid leftClawArm = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    private DoubleSolenoid rightClawArm = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 4);

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
    private ClawModes state = ClawModes.clawOff;

    public static Claw getInstance() {
        if (instance == null) {
            instance = new Claw();
        }
        return instance;
    }

    public void open() {
        state = ClawModes.clawOpen;
    }
    public void close() {
        state = ClawModes.clawClosed;
    }
    public void off() {
        state = ClawModes.clawOff;
    }

    public void update() {
        leftClawArm.set(state.value);
        rightClawArm.set(state.value);
    }

    public boolean isOpen() {
        return state == ClawModes.clawOpen;
    }

    public boolean isClosed() {
        return state == ClawModes.clawClosed;
    }
}
