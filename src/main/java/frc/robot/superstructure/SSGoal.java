package frc.robot.superstructure;

public enum SSGoal {
    STOW(0.0, 0.0, 0.0),
    GROUND_INTAKE_CONE(0.0, 0.0, 0.0),
    GROUND_INTAKE_CUBE(0.0, 0.0, 0.0),
    SHELF_INTAKE_CONE(0.0, 0.0, 0.0),
    SHELF_INTAKE_CUBE(0.0, 0.0, 0.0),
    LOW_SCORE(0.0, 0.0, 0.0),
    MID_CUBE_SCORE(0.0, 0.0, 0.0),
    MID_CONE_SCORE(0.0, 0.0, 0.0),
    HIGH_CUBE_SCORE(0.0, 0.0, 0.0),
    HIGH_CONE_SCORE(0.0, 0.0, 0.0),
    CLIMB_STAGE_1(0.0, 0.0, 0.0), //extend forks
    CLIMB_STAGE_2(0.0, 0.0, 0.0) //curl
    ;

    public final double arm;
    public final double elevator;
    public final double wrist;

    SSGoal(double arm, double elevator, double wrist) {
        this.arm = arm;
        this.elevator = elevator;
        this.wrist = wrist;
    }
}