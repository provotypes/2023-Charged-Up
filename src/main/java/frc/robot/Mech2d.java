package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Mech2d {
    private final Mechanism2d canvas = new Mechanism2d(3,3);
    private final MechanismRoot2d root = canvas.getRoot("Elevator Root", .5, .5);
    private final MechanismLigament2d elevator = root.append(new MechanismLigament2d("elevator", ELEVATOR_DOWN_HEIGHT, 90, 4, new Color8Bit(Color.kFirstRed)));
    private final MechanismLigament2d arm = elevator.append(new MechanismLigament2d("arm", 1.2234926, -Arm.ArmPosition.armInside.value+180, 2, new Color8Bit(Color.kCoral))); //lengeth in meters
    private final MechanismLigament2d robotBase = root.append(new MechanismLigament2d("robot base", 0.8128, 0, 1, new Color8Bit(Color.kBlueViolet)));

    // not sure about either of these measurements, should check them
    // in meters
    private static final double ELEVATOR_UP_HEIGHT = 1.2777579982;
    private static final double ELEVATOR_DOWN_HEIGHT = 1.97;
    

    public Mech2d() {
        Robot.mainTab.add("Test Mech", canvas);
    }

    public void updateElevator(double height) {
        elevator.setLength(height);
    }

    public void updateArm(double angle) {
        arm.setAngle(180 + angle);
    }

}
