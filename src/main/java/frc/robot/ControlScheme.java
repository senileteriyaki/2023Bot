package frc.robot;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.PathingMode;
import frc.robot.subsystems.drive.PathingOverride;
import frc.robot.subsystems.drive.SwerveInput;
import frc.robot.superstructure.SS;
import frc.robot.superstructure.SS.Flag;
import frc.robot.util.Util;


public class ControlScheme {

    private final SS ss;
    private final Drive drive;

    public ControlScheme(SS ss, Drive drive) {
        this.ss = ss;
        this.drive = drive;
    }

    public void init() {
        drive.queueState(PathingMode.FIELD_RELATIVE);
        drive.setPathingOverride(PathingOverride.NONE);
        System.out.println("Controls initialized");
    }

    public void update() {
        double x_ = OI.deadband(-OI.DR.getLeftY());
        double y_ = OI.deadband(-OI.DR.getLeftX());
        double w_ = 0.5 * -Util.sqInput(OI.deadband(OI.DR.getRightX()));
        double throttle = Util.sqInput(
                1.0 - OI.deadband(Math.max(OI.DR.getLeftTriggerAxis(), OI.DR.getRightTriggerAxis())));

        if (OI.DR.getPOV() == 180) {
            drive.zeroGyro();
        }
        drive.setInput(new SwerveInput(x_, y_, w_, throttle));

        if (OI.DR.getRightTriggerAxis() > 0.5) {
            drive.setPathingOverride(PathingOverride.TRACKING);
        } else {
            drive.setPathingOverride(PathingOverride.NONE);
        }

        ss.set(Flag.HOME, OI.DR.getBackButton());
        ss.set(Flag.MANUAL_UP, OI.DR.getAButton());
        ss.set(Flag.MANUAL_DOWN, OI.DR.getBButton());
        ss.set(Flag.SCORE_LOW, OI.DR.getXButton());
        ss.set(Flag.SCORE_HIGH, OI.DR.getYButton());

    }
}
