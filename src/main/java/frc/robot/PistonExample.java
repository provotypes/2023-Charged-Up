// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/** Add your docs here. */
public class PistonExample {

    private static PistonExample instance;
    private DoubleSolenoid bigPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    private DoubleSolenoid smallPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 4);

    private PistonExample() {
    }

    private enum PistonModes {
        pistonOut (Value.kForward),
        pistonIn (Value.kReverse),
        pistonOff (Value.kOff);

        private Value value;

        private PistonModes(Value value) {
            this.value = value;
        }
    }
    private PistonModes state = PistonModes.pistonOff;

    public static PistonExample getInstance() {
        if (instance == null) {
            instance = new PistonExample();
        }
        return instance;
    }

    public void out() {
        state = PistonModes.pistonOut;
    }
    public void in() {
        state = PistonModes.pistonIn;
    }
    public void off() {
        state = PistonModes.pistonOff;
    }

    public void update() {
        bigPiston.set(state.value);
    }
}
