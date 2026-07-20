package frc.robot.subsystems.drive;

import frc.robot.util.IState;

public enum PathingMode implements IState {
    DISABLED,
    FIELD_RELATIVE,
    POSE_FOLLOWING,
    PATH_FOLLOWING,
    SHOOTING
}
