package frc.robot.superstructure;

import java.util.function.BooleanSupplier;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;

public class SSGoal2 {
    public static enum SUBSYSTEMS{
        ELEV,
        ARM,
        WRIST
    }

    public final double arm; //positions of the subsystems
    public final double wrist;
    public final double elev;
    public final BooleanSupplier armDone; //suppliers for when they're done, usually atTarget if we need to wait, x -> true if not
    public final BooleanSupplier elevDone;
    public final BooleanSupplier wristDone;
    public final SUBSYSTEMS[] order; //order in which to move the subsystems

    public SSGoal2(double arm, BooleanSupplier armDone,  double elev, BooleanSupplier elevDone, double wrist, BooleanSupplier wristDone,
            SUBSYSTEMS subsystem1, SUBSYSTEMS subsystem2, SUBSYSTEMS subsystem3){
        this.arm = arm;
        this.elev = elev;
        this.wrist = wrist;
        this.armDone = armDone;
        this.wristDone = wristDone;
        this.elevDone = elevDone;
        this.order = new SUBSYSTEMS[]{subsystem1, subsystem2, subsystem3};
    }

    public SSGoal2(double arm, double elev, double wrist){
        this.arm = arm;
        this.elev = elev;
        this.wrist = wrist;
        armDone = () -> true;
        wristDone = () -> true;
        elevDone = () -> true;
        order = new SUBSYSTEMS[]{ARM, ELEV, WRIST};
    }


}