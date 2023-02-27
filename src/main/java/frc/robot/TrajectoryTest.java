package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Timer;

public class TrajectoryTest {
    DifferentialDriveVoltageConstraint autoVoltageConstraint;
    double ksVolts = 0.22;
    double kvVoltSecondsPerMeter = 1.98;
    double kaVoltSecondsSquaredPerMeter = 0.2;

    double kMaxSpeedMetersPerSecond = 3;
    double kMaxAccelerationMetersPerSecondSquared = 1;

    Trajectory exampleTrajectory;
    RamseteController ramseteController;

    Timer trajectoryTimer = new Timer();

    final DifferentialDriveKinematics kinematics = DriveTrain.getInstance().kinematics;

    public void trajectoryInit() {
        autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                ksVolts, 
                kvVoltSecondsPerMeter, 
                kaVoltSecondsSquaredPerMeter), 
            DriveTrain.getInstance().kinematics, 
            10);

        TrajectoryConfig config = new TrajectoryConfig(kMaxSpeedMetersPerSecond, kaVoltSecondsSquaredPerMeter).setKinematics(kinematics);

        // An example trajectory to follow.  All units in meters.
        exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                // Pass config
                config);

        ramseteController = new RamseteController();
        trajectoryTimer.reset();
        trajectoryTimer.start();
        
    }

    public void trajectoryPeriodic() {
        Trajectory.State goal =  exampleTrajectory.sample(trajectoryTimer.get());
        ChassisSpeeds adjustedSpeeds = ramseteController.calculate(DriveTrain.getInstance().field.getRobotPose(), goal);
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(adjustedSpeeds);

        double left = wheelSpeeds.leftMetersPerSecond;
        double right = wheelSpeeds.rightMetersPerSecond;

        DriveTrain.getInstance().tankDriveMetersPerSecond(left, right);
    }
}
