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
    private final SendableChooser<Routine> routinePicker = new SendableChooser<Routine>();
    private final SendableChooser<GridColumn> gridColumnPicker = new SendableChooser<GridColumn>();
    private final SendableChooser<GridColumn> gridColumnPicker2 = new SendableChooser<GridColumn>();
    private final SendableChooser<GridRow> gridRowPicker = new SendableChooser<GridRow>();
    private final SendableChooser<GridRow> gridRowPicker2 = new SendableChooser<GridRow>();
    private final SendableChooser<GamePiecePosition> gamePiecePicker = new SendableChooser<GamePiecePosition>();
    private final SendableChooser<GamePiecePosition> gamePiecePicker2 = new SendableChooser<GamePiecePosition>();
    private final SendableChooser<DockingPosition> dockingPosPicker = new SendableChooser<DockingPosition>();

    private AutoRoutine() {
        boolean firstTry = true;
        for (Routine option : Routine.values()) { //TODO: maybe add a default with SendableChooser.setDefaultOption()
            routinePicker.addOption(option.toString(), option);
        }
        autoTab.add("Routine", routinePicker).withSize(2, 1);

        for (GridColumn column : GridColumn.values()) {
            gridColumnPicker.addOption(column.toString(), column);
            gridColumnPicker2.addOption(column.toString(), column);
        }
        autoTab.add("Grid Column", gridColumnPicker);
        autoTab.add("Grid Column 2", gridColumnPicker2);

        for (GridRow row : GridRow.values()) {
            gridRowPicker.addOption(row.toString(), row);
            gridRowPicker2.addOption(row.toString(), row);
        }
        autoTab.add("Grid Row", gridRowPicker);
        autoTab.add("Grid Row 2", gridRowPicker2);

        for (GamePiecePosition position : GamePiecePosition.values()) {
            gamePiecePicker.addOption(position.toString(), position);
            gamePiecePicker2.addOption(position.toString(), position);
        }
        autoTab.add("Game Piece", gamePiecePicker);
        autoTab.add("Game Piece 2", gamePiecePicker2);

        for (DockingPosition position : DockingPosition.values()) {
            dockingPosPicker.addOption(position.toString(), position);
        }
        autoTab.add("Docking Position", dockingPosPicker);

        // Shuffleboard.selectTab(autoTab.getTitle()); //brings up the auto tab; not necessary but nice
    }

    public static AutoRoutine getInstance() {
        if (instance == null) {
            instance = new AutoRoutine();
        }
        return instance;
    }
 // 11ft 9in - (2ft 1 3/4in) - 5*(1ft 6 1/4in) / 7
 // 141 - 25.75 - 91.25
 // 3.5"

    public enum Routine {
        None,
        DockOnly,
        GamePieceOnly,
        GamePieceAndDock,
        DoubleGamePiece,
        DoubleGamePieceAndDock;
    }

    public Routine routine = Routine.None;

    public enum GridColumn {
        LeftLeft (20.19, 196.19, 48.0),
        LeftMiddle (42.19, 174.19, 48.0),
        LeftRight (64.19, 152.19, 48.0),
        MiddleLeft (86.19, 130.19, 48.0),
        MiddleMiddle (108.19, 108.19, 48.0),
        MiddleRight (130.19, 86.19, 48.0),
        RightLeft (152.19, 64.19, 48.0),
        RightMiddle (174.19, 42.19, 48.0),
        RightRight (196.19, 20.19, 48.0);

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
    public GridColumn gridColumn2 = GridColumn.LeftRight;

    public enum GridRow {
        High (311.11),
        Middle (294.11),
        Low (280.11);

        public double y;

        private GridRow(double y) {
            this.y = y;
        }
    }
    public GridRow gridRow = GridRow.High;
    public GridRow gridRow2 = GridRow.High;


    public enum GamePiecePosition {
        Position1 (-47.36, 47.36, 36.19),
        Position2 (-47.36, 47.36, 84.19),
        Position3 (-47.36, 47.36, 132.19),
        Position4 (-47.36, 47.36, 180.19);

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
        Position1 (-174.61, 174.61, 39.39),
        Position2 (-174.61, 174.61, 63.39),
        Position3 (-174.61, 174.61, 87.39);

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
            case 1 -> {
                gamePiecePosition = gamePiecePosition2;
                gridColumn = gridColumn2;
                gridRow = gridRow2;
                step2++;
            }
            case 2 -> {
                gamePieceOnly();
                if (step == 9) {
                    step2++;
                    step = 0;
                }
            }
            case 3 -> {
                if (routine == Routine.DoubleGamePieceAndDock) {
                    dockOnly();
                }
                else {
                    step2++;
                }
            }
        }
    }

    public void initAuto() {
        routine = routinePicker.getSelected();
        gamePiecePosition = gamePiecePicker.getSelected();
        gamePiecePosition2 = gamePiecePicker2.getSelected();
        dockingPosition = dockingPosPicker.getSelected();
        gridColumn = gridColumnPicker.getSelected();
        gridColumn2 = gridColumnPicker2.getSelected();
        gridRow = gridRowPicker.getSelected();
        gridRow2 = gridRowPicker2.getSelected();
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
            case DoubleGamePiece -> {
                doubleGamePiece();
            }
            case DoubleGamePieceAndDock -> {
                doubleGamePiece();
            }
            default -> {
                return;
            }
        }
    }

}
