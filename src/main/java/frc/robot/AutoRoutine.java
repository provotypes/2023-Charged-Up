package frc.robot;


import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import static frc.robot.Robot.mainTab;


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
    private int step2 = 0;
    //private int tick = 0;

    private final ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
    private final SendableChooser<String> routinePicker = new SendableChooser<String>();
    private final SendableChooser<String> gridColumnPicker = new SendableChooser<String>();
    private final SendableChooser<String> gridRowPicker = new SendableChooser<String>();
    private final SendableChooser<String> gamePiecePicker = new SendableChooser<String>();
    private final SendableChooser<String> dockingPosPicker = new SendableChooser<String>();

    private AutoRoutine() {
        boolean firstTry = true;
        for (Routine option : Routine.values()) { //TODO: maybe add a default with SendableChooser.setDefaultOption()
            routinePicker.addOption(option.toString(), option.name());
        }
        autoTab.add("Routine", routinePicker).withSize(2, 1);

        for (GridColumn column : GridColumn.values()) {
            gridColumnPicker.addOption(column.toString(), column.name());
        }
        autoTab.add("Grid Column", gridColumnPicker);

        for (GridRow row : GridRow.values()) {
            gridRowPicker.addOption(row.toString(), row.name());
        }
        autoTab.add("Grid Row", gridRowPicker);

        for (GamePiecePosition position : GamePiecePosition.values()) {
            gamePiecePicker.addOption(position.toString(), position.name());
        }
        autoTab.add("Game Piece", gamePiecePicker);

        for (DockingPosition position : DockingPosition.values()) {
            dockingPosPicker.addOption(position.toString(), position.name());
        }
        autoTab.add("Docking Position", dockingPosPicker);

        Shuffleboard.selectTab(autoTab.getTitle()); //brings up the auto tab; not necessary but nice
    }

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
    public GamePiecePosition gamePiecePosition2 = GamePiecePosition.Position2;

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

    private boolean moveBackwardsTo(double targetX, double targetY) {
        if (turnTo(getAngleTo(targetX, targetY) + 180)) {
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
                
                if (moveClawTo((Robot.alliance == Alliance.Blue ? gamePiecePosition.blueX : gamePiecePosition.redX), gamePiecePosition.y)) {
                    step++;
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
                if (moveTo((Robot.alliance == Alliance.Blue ? gridColumn.blueX + gridColumn.lineupOffset : gridColumn.redX - gridColumn.lineupOffset), gridRow.y)) {
                    step++;
                }
            }
            case 4 -> {
                if (gridRow == GridRow.High) {
                    if (!arm.isAtPosition(Arm.ArmPosition.armHigh)) {
                        arm.clawHigh();
                    }
                    else {
                        step++;
                    }
                }
                else if (gridRow == GridRow.Middle) {
                    if (!arm.isAtPosition(Arm.ArmPosition.armLow)) {
                        arm.clawLow();
                    }
                    else {
                        step++;
                    }
                }
                else {
                    if (!arm.isAtPosition(Arm.ArmPosition.armPickupFloor)) {
                        arm.clawPickupFloor();
                    }
                    else {
                        step++;
                    }
                }
            }
            case 5 -> {
                if (moveTo((Robot.alliance == Alliance.Blue ? gridColumn.blueX : gridColumn.redX), gridRow.y)) {
                    step++;
                }
            }
            case 6 -> {
                if (!claw.isOpen()) {
                    claw.open();
                }
                else {
                    step++;
                }
            }
            case 7 -> {
                if (moveBackwardsTo((Robot.alliance == Alliance.Blue ? gridColumn.blueX + gridColumn.lineupOffset : gridColumn.redX - gridColumn.lineupOffset), gridRow.y)) {
                    step++;
                }
            }
            case 8 -> {
                if (!arm.isAtPosition(Arm.ArmPosition.armPickupFloor)) {
                    arm.clawPickupFloor();
                }
                else {
                    step++;
                }
            }
        }
    }

    private void gamePieceAndDock() {
        switch (step2) {
            case 0 -> {
                gamePieceOnly();
                if (step == 9) {
                    step2++;
                    step = 0;
                }
            }
            case 1 -> {
                dockOnly();
            }
        }
    }

    private void doubleGamePiece() {
        switch (step2) {
            case 0 -> {
                gamePieceOnly();
                if (step == 9) {
                    step2++;
                    step = 0;
                }
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
