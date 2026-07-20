package frc.robot.subsystems.drive;

import frc.robot.util.IState;

public enum PathingOverride implements IState {
    NONE,
    INTAKING,
    TRACKING,
    BASELOCK,
    SHOOTING
}
