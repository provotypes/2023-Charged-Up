package frc.robot;


import edu.wpi.first.wpilibj.DriverStation.Alliance;


public class AutoRoutine {
    private static AutoRoutine instance;

    private final double turnSpeed = 0.5;
    private LimelightVisionTracking limelight = LimelightVisionTracking.getInstance();
    private SeesawAuto seesawAuto = SeesawAuto.getInstance();
    private DriveTrain drivetrain = DriveTrain.getInstance();

    private int step = 0;
    private int tick = 0;

    private AutoRoutine() {}
    public static AutoRoutine getInstance() {
        if (instance == null) {
            instance = new AutoRoutine();
        }
        return instance;
    }


    public enum Routine {
        None,
        DockOnly,
        GamePiece1;
    }

    public Routine routine = Routine.None;

    public enum DockingPosition {
        Position1 (0.0, 0.0, 0.0),
        Position2 (0.0, 0.0, 0.0),
        Position3 (0.0, 0.0, 0.0);

        public double blueX;
        public double redX;
        public double y;

        private DockingPosition(double blueX, double redX, double y) {
            this.blueX = blueX;
            this.redX = redX;
            this.y = y;
        }
    }

    public DockingPosition dockingPosition = DockingPosition.Position1;

    private boolean moveTo(double targetX, double targetY) {
        if (turnTo(getAngleTo(targetX, targetY))) {

            double x = drivetrain.getX();
            double y = drivetrain.getY();

            if (x == targetX && y == targetY) {
                return true;
            }
            double distance = Math.sqrt(Math.pow(targetX - x, 2) + Math.pow(targetY - y, 2));
            // TODO: double, double, toil, and trouble! (william do code for driving forward)
            
        }
        return false;
    }

    private boolean turnTo(double targetRotation) {
        double currentRotation = drivetrain.getRotation();
        double deltaRotation = Math.min(-turnSpeed, Math.max(targetRotation - currentRotation, turnSpeed));

        if (deltaRotation == 0) {
            return true;
        }

        // turn by deltaRotation

        return false;
    }

    private double getAngleTo(double targetX, double targetY) {

        double x = drivetrain.getX();
        double y = drivetrain.getY();

        double dx = targetX - x;
        double dy = targetY - y;

        if (dx == 0) {
            if (dy == 0) {
                return drivetrain.getRotation();
            }
            if (dy > 0) {
                return 270;
            }
            return 90;
        }
        if (dy == 0) {
            if (dx > 0) {
                return 0;
            }
            return 180;
        }

        return Math.toDegrees(Math.atan(dy/dx));
    }

    private void dockOnly() {
        switch (step) {
            case 0 -> {
                if (moveTo(drivetrain.getX(), dockingPosition.y)) {
                    step++;
                }
            }
            case 1 -> {
                if (Robot.alliance == Alliance.Blue) {
                    if (moveTo(dockingPosition.blueX, dockingPosition.y)) {
                        step++;
                    }
                }
                else {
                    if (moveTo(dockingPosition.redX, dockingPosition.y)) {
                        step++;
                    }
                }
            }
            case 2 -> {
                seesawAuto.autoPark();
            }
        }

    }

    public void doRoutine() {
        switch (routine) {
            case DockOnly -> {
                dockOnly();
            }
            default -> {
                return;
            }
        }
    }

}
