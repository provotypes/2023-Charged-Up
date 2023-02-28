package frc.robot;


import edu.wpi.first.wpilibj.DriverStation.Alliance;


public class AutoRoutine {
    private static AutoRoutine instance;

    private final double turnSpeed = 0.5;

    private final double clawOffset = 24.0;
    // TODO: measure from center of robot to middle of claw's pickup space

    //private LimelightVisionTracking limelight = LimelightVisionTracking.getInstance();
    private SeesawAuto seesawAuto = SeesawAuto.getInstance();
    private DriveTrain drivetrain = DriveTrain.getInstance();
    private Arm arm = Arm.getInstance();
    private Claw claw = Claw.getInstance();

    private int step = 0;
    //private int tick = 0;

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
        GamePieceOnly,
        GamePieceAndDock;
    }

    public Routine routine = Routine.None;

    public enum GridColumn {
        LeftLeft (0.0, 0.0, 0.0),
        LeftMiddle (0.0, 0.0, 0.0),
        LeftRight (0.0, 0.0, 0.0),
        MiddleLeft (0.0, 0.0, 0.0),
        MiddleMiddle (0.0, 0.0, 0.0),
        MiddleRight (0.0, 0.0, 0.0),
        RightLeft (0.0, 0.0, 0.0),
        RightMiddle (0.0, 0.0, 0.0),
        RightRight (0.0, 0.0, 0.0);

        public double blueX;
        public double redX;
        public double lineupOffset;

        private GridColumn(double blueX, double redX, double lineupOffset) {
            this.blueX = blueX;
            this.redX = redX;
            this.lineupOffset = lineupOffset;
        }

    }
    public GridColumn gridColumn = GridColumn.LeftLeft;

    public enum GridRow {
        High (0.0),
        Middle (0.0),
        Low (0.0);

        public double y;

        private GridRow(double y) {
            this.y = y;
        }
    }
    public GridRow gridRow = GridRow.High;


    public enum GamePiecePosition {
        Position1 (0.0, 0.0, 0.0),
        Position2 (0.0, 0.0, 0.0),
        Position3 (0.0, 0.0, 0.0),
        Position4 (0.0, 0.0, 0.0);

        public double blueX;
        public double redX;
        public double y;

        private GamePiecePosition(double blueX, double redX, double y) {
            this.blueX = blueX;
            this.redX = redX;
            this.y = y;
        }
    }

    public GamePiecePosition gamePiecePosition = GamePiecePosition.Position1;

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

    /**
     * Moves the robot untill the claw is aligned with the target position
     */
    private boolean moveClawTo(double targetX, double targetY) {
        
        double x = drivetrain.getX();
        double y = drivetrain.getY();
        double yaw = drivetrain.getRotation();

        double distance = Math.sqrt(Math.pow(targetX - x, 2) + Math.pow(targetY - y, 2));

        double actualDistance = distance - clawOffset;
        
        double actualX = x + (Math.cos(Math.toRadians(yaw)) * actualDistance);
        double actualY = y + (-Math.sin(Math.toRadians(yaw)) * actualDistance);
        // invert result of sin() because yaw is clockwise+ but math is clockwise-

        return moveTo(actualX, actualY);
    }

    /**
     * Moves the robot to a position on the field.
     */
    private boolean moveTo(double targetX, double targetY) {
        if (turnTo(getAngleTo(targetX, targetY))) {

            double x = drivetrain.getX();
            double y = drivetrain.getY();

            if (x == targetX && y == targetY) {
                return true;
            }
            double distance = Math.sqrt(Math.pow(targetX - x, 2) + Math.pow(targetY - y, 2));

            // controls max speed, and when the robot starts to slow down
            double driveSpeed = Math.max(0.72, distance / 24.0);

            drivetrain.arcadeDrive(driveSpeed, 0.0);
            
        }
        return false;
    }

    private boolean turnTo(double targetRotation) {
        double currentRotation = drivetrain.getRotation();
        double deltaRotation = Math.min(-turnSpeed, Math.max(targetRotation - currentRotation, turnSpeed));

        if (deltaRotation == 0) {
            return true;
        }

        drivetrain.arcadeDrive(0.0, deltaRotation);

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
    
    private void gamePieceOnly() {
        switch (step) {
            case 0 -> { // Move to the game piece, simultaneously move arm to pickup position
                if (!arm.isAtPosition(Arm.ArmPosition.armPickupFloor)) {
                    arm.clawPickupFloor();
                }
                // assume the robot is set up with a straight shot to the selected game piece
                if (Robot.alliance == Alliance.Blue) {
                    if (moveClawTo(gamePiecePosition.blueX, gamePiecePosition.y)) {
                        step++;
                    }
                }
                else {
                    if (moveClawTo(gamePiecePosition.redX, gamePiecePosition.y)) {
                        step++;
                    }
                }
            }
            case 1 -> { // let arm finish moving if necessary
                arm.clawPickupFloor();
                if (arm.isAtPosition(Arm.ArmPosition.armPickupFloor)) {
                    step++;
                }
            }
            case 2 -> { // close the claw around the game piece
                if (!claw.isClosed()) {
                    claw.close();
                }
                else {
                    step++;
                }
            }
            case 3 -> {
                if (!arm.isAtPosition(Arm.ArmPosition.armTransport)) {
                    arm.clawTransport();
                }
                if (Robot.alliance == Alliance.Blue) {
                    if (moveTo(gridColumn.blueX + gridColumn.lineupOffset, gridRow.y)) {}
                }
            }
            case 4 -> {
                
            }
        }
    }

    private void gamePieceAndDock() {
        switch (step) {
            case 0 -> {

            }
        }
    }

    public void doRoutine() {
        switch (routine) {
            case DockOnly -> {
                dockOnly();
            }
            case GamePieceOnly -> {
                gamePieceOnly();
            }
            case GamePieceAndDock -> {
                gamePieceAndDock();
            }
            default -> {
                return;
            }
        }
    }

}
