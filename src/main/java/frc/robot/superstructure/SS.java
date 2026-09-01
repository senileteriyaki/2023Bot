package frc.robot.superstructure;

import frc.robot.subsystems.SubsystemBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.wrist.Wrist;
import static frc.robot.superstructure.SSGoal2.SUBSYSTEMS.*;

import java.util.EnumSet;

import org.littletonrobotics.junction.Logger;

public class SS extends SubsystemBase<SS.Command> {

    public enum Flag {
        HOME,
        INTAKE_GROUND,
        INTAKE_SHELF,
        SCORE_LOW,
        SCORE_MID,
        SCORE_HIGH,
        WANTS_CUBE,
        MANUAL_UP,
        MANUAL_DOWN,
        PRECLIMB,
        CLIMB
    }

    public enum Command {
        IDLE,
        HOMING,
        INTAKING_GROUND,
        INTAKING_SHELF,
        SCORE_LOW,
        SCORE_MID,
        SCORE_HIGH,
        MANUAL_CONTROL,
        CLIMB_PLACE,
        CLIMB_CURL,
        STOWING
    }

    private static final double MANUAL_VOLTS = 2.0;

    private static SS instance;

    private final EnumSet<Flag> flags = EnumSet.noneOf(Flag.class);

    private final Drive drive;
    private final Arm arm;
    private final Wrist wrist;
    private final LED led;
    private final EndEffector ee;
    private final Elevator elevator;
    private boolean isCube = true;


    private SSGoal2 currentAchieveGoal = null;
    private SSGoal2.SUBSYSTEMS currState = null;
    private int numState = 0;

    public static SSGoal2 STOW;
    public static SSGoal2 GROUND_INTAKE_CONE;
    public static SSGoal2 GROUND_INTAKE_CUBE;
    public static SSGoal2 SHELF_INTAKE_CONE;
    public static SSGoal2 SHELF_INTAKE_CUBE;
    public static SSGoal2 LOW_SCORE;
    public static SSGoal2 MID_CONE_SCORE;
    public static SSGoal2 MID_CUBE_SCORE;
    public static SSGoal2 HIGH_CONE_SCORE;
    public static SSGoal2 HIGH_CUBE_SCORE;
    public static SSGoal2 CLIMB_STAGE_1;

    public static SSGoal2 CLIMB_STAGE_2;

    public static SS getInstance() {
        if (instance == null) {
            instance = new SS();
        }
        return instance;
    }

    private SS() {
        super("Superstructure");
        drive = Drive.getInstance();
        arm = Arm.getInstance();
        elevator = Elevator.getInstance();
        led = LED.getInstance();
        ee = EndEffector.getInstance();
        wrist = Wrist.getInstance();

        setCommand(Command.IDLE);

        HIGH_CONE_SCORE = new SSGoal2(
                40.0, arm::atTarget,
                1.60, () -> true,
                35.0, () -> true,
                ARM, ELEV, WRIST);

        STOW = new SSGoal2(
                0.0, () -> true,
                0.0, () -> true,
                0.0, () -> true,
                WRIST, ARM, ELEV);

        GROUND_INTAKE_CONE = new SSGoal2(
                -20.0, () -> true,
                0.05, () -> true,
                -10.0, () -> true,
                ARM, WRIST, ELEV);

        GROUND_INTAKE_CUBE = new SSGoal2(
                -25.0, () -> true,
                0.05, () -> true,
                0.0, () -> true,
                ARM, WRIST, ELEV);

        SHELF_INTAKE_CONE = new SSGoal2(
                10.0, () -> true,
                1.00, () -> true,
                0.0, () -> true,
                ELEV, ARM, WRIST);
        SHELF_INTAKE_CUBE = new SSGoal2(
                10.0, () -> true,
                1.00, () -> true,
                5.0, () -> true,
                ELEV, ARM, WRIST);

        LOW_SCORE = new SSGoal2(
                -15.0, () -> true,
                0.10, () -> true,
                -5.0, () -> true,
                ELEV, WRIST, ARM);

        MID_CONE_SCORE = new SSGoal2(
                25.0, () -> true,
                0.90, () -> true,
                20.0, () -> true,
                ARM, ELEV, WRIST);

        MID_CUBE_SCORE = new SSGoal2(
                20.0, () -> true,
                0.85, () -> true,
                10.0, () -> true,
                ARM, ELEV, WRIST);

        HIGH_CUBE_SCORE = new SSGoal2(
                35.0, arm::atTarget,
                1.55, () -> true,
                15.0, () -> true,
                ARM, ELEV, WRIST);

        CLIMB_STAGE_1 = new SSGoal2(
                50.0, () -> true,
                0.60, () -> true,
                45.0, () -> true,
                ELEV, ARM, WRIST);

        CLIMB_STAGE_2 = new SSGoal2(
                -10.0, () -> true,
                0.00, () -> true,
                -20.0, () -> true,
                ARM, WRIST, ELEV);
    }

    public void enable(Flag flag) {
        flags.add(flag);
    }

    public void disable(Flag flag) {
        flags.remove(flag);
    }

    public void set(Flag flag, boolean active) {
        if (active) {
            flags.add(flag);
        } else {
            flags.remove(flag);
        }
    }

    public void toggle(Flag flag) {
        set(flag, !has(flag));
    }

    public boolean has(Flag flag) {
        return flags.contains(flag);
    }

    @Override
    protected void inputPeriodic() {
    }

    @Override
    protected void outputPeriodic() {
        String[] active = flags.stream().map(Enum::name).toArray(String[]::new);
        Logger.recordOutput("Superstructure/Flags", active);
        
    }

    @Override
    protected void handle() {

        if (has(Flag.HOME)) {
            setCommand(Command.HOMING);
        } else if (has(Flag.MANUAL_UP) || has(Flag.MANUAL_DOWN)) {
            setCommand(Command.MANUAL_CONTROL);
        } else if (has(Flag.PRECLIMB)) {
            setCommand(Command.CLIMB_PLACE);
        } else if (has(Flag.CLIMB)) {
            setCommand(Command.CLIMB_CURL);
        } else if (has(Flag.SCORE_HIGH)) {
            setCommand(Command.SCORE_HIGH);
        } else if (has(Flag.SCORE_MID)) {
            setCommand(Command.SCORE_MID);
        } else if (has(Flag.SCORE_LOW)) {
            setCommand(Command.SCORE_LOW);
        } else if (has(Flag.INTAKE_SHELF)) {
            setCommand(Command.INTAKING_SHELF);
        } else if (has(Flag.INTAKE_GROUND)) {
            setCommand(Command.INTAKING_GROUND);
        } else {
            setCommand(Command.STOWING);
        }


        isCube = has(Flag.WANTS_CUBE);

        switch (getCommand()) {
            case IDLE:
                achieve(STOW);
                ee.setCommand(EndEffector.Command.IDLE);
                break;

            case HOMING:
                elevator.home();
                arm.home();
                wrist.home();
                ee.setCommand(EndEffector.Command.DISABLED);
                break;

            case INTAKING_GROUND:
                achieve(isCube ? GROUND_INTAKE_CUBE : GROUND_INTAKE_CONE);
                if (achieved()) {
                    runEndEffectorIntakeMode();
                }
                led.setCommand(LED.Command.INTAKE);
                break;

            case INTAKING_SHELF:
                achieve(isCube ? SHELF_INTAKE_CUBE : SHELF_INTAKE_CONE);
                if (achieved()) {
                    runEndEffectorIntakeMode();
                }
                led.setCommand(LED.Command.INTAKE);
                break;

            case SCORE_LOW:
                achieve(LOW_SCORE);
                if (achieved()) {
                    runEndEffectorScoringMode();
                }
                led.setCommand(LED.Command.OUTTAKE);
                break;

            case SCORE_MID:
                achieve(isCube ? MID_CUBE_SCORE : MID_CONE_SCORE);
                if (achieved()) {
                    runEndEffectorScoringMode();
                }
                led.setCommand(LED.Command.SCORING);

                break;

            case SCORE_HIGH:
                achieve(isCube ? HIGH_CUBE_SCORE : HIGH_CONE_SCORE);
                if (achieved()) {
                    runEndEffectorScoringMode();
                }
                led.setCommand(LED.Command.SCORING);
                break;

            case MANUAL_CONTROL:
                elevator.setManualVoltage(has(Flag.MANUAL_UP) ? MANUAL_VOLTS : -MANUAL_VOLTS);
                ee.setCommand(EndEffector.Command.IDLE);
                break;
            case CLIMB_PLACE:
                achieve(CLIMB_STAGE_1);
                ee.setCommand(EndEffector.Command.IDLE);
                led.setCommand(LED.Command.CLIMBING);
                break;
            case CLIMB_CURL:
                if (!achieved()) {
                    setCommand(Command.CLIMB_PLACE);
                }
                achieve(CLIMB_STAGE_2);
                ee.setCommand(EndEffector.Command.IDLE);
                led.setCommand(LED.Command.CLIMBING);
                break;
            case STOWING:
                achieve(STOW);
                ee.setCommand(EndEffector.Command.IDLE);
                break;
        }
    }

    private void runEndEffectorScoringMode() {
        if (has(Flag.SCORE_LOW) || has(Flag.SCORE_MID) || has(Flag.SCORE_HIGH)) {
            ee.setCommand(isCube ? EndEffector.Command.OUTTAKE_CUBE : EndEffector.Command.OUTTAKE_CONE);
        } else {
            ee.setCommand(EndEffector.Command.IDLE);
        }
    }

    private void runEndEffectorIntakeMode() {
        if (has(Flag.INTAKE_GROUND) || has(Flag.INTAKE_SHELF)) {
            ee.setCommand(isCube ? EndEffector.Command.INTAKE_CUBE : EndEffector.Command.INTAKE_CONE);
        }
    }

    /*
     * private void achieve(SSGoal goal) {
     * arm.setTarget(goal.arm);
     * elevator.setTarget(goal.elevator);
     * wrist.setTarget(goal.wrist);
     * }
     */

    private void achieve(SSGoal2 goal) {
        if (goal != currentAchieveGoal) {
            currentAchieveGoal = goal;
            numState = 0;
            currState = goal.order[0];
        }

        switch (currState) {
            case ARM:
                arm.trackToAngle(goal.arm);
                if (goal.armDone.getAsBoolean() && numState < goal.order.length - 1) {
                    numState += 1;
                    currState = goal.order[numState];
                }
                break;
            case ELEV:
                elevator.trackToHeight(goal.elev);
                if (goal.elevDone.getAsBoolean() && numState < goal.order.length - 1) {
                    numState += 1;
                    currState = goal.order[numState];
                }
                break;
            case WRIST:
                wrist.trackToAngle(goal.wrist);
                if (goal.wristDone.getAsBoolean() && numState < goal.order.length - 1) {
                    numState += 1;
                    currState = goal.order[numState];
                }
                break;
        }

    }

    private boolean achieved() {
        return arm.atTarget() && elevator.atTarget() && wrist.atTarget();
    }
}