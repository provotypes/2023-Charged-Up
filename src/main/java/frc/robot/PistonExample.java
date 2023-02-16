// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.ModuleLayer.Controller;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/** Add your docs here. */
public class PistonExample {

    private DoubleSolenoid bigPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,     0, 1);
    private DoubleSolenoid smallPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 4);

    private final XboxController xboxController = new XboxController(0);

    public void teleopPeriodic() {

        if (xboxController.getPOV() == 0) {
            bigPiston.set(Value.kForward);
        }
        else if (xboxController.getPOV() == 180) {
            bigPiston.set(Value.kReverse);
        }
        else {
            bigPiston.set(Value.kOff);
        }

        if (xboxController.getPOV() == 90) {
            smallPiston.set(Value.kForward);
        }
        else if (xboxController.getPOV() == 270) {
            smallPiston.set(Value.kReverse);
        }
        else {
            smallPiston.set(Value.kOff);
        }
    }
}
