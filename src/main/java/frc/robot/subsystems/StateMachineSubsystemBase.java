// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.PerfTracker;
import frc.robot.util.CustomTimer;
import frc.robot.util.IState;
import frc.robot.util.IStateMachine;
import org.littletonrobotics.junction.Logger;

/** States are now handled inside the respective subsystem */
public abstract class StateMachineSubsystemBase<T extends Enum<T> & IState> implements IStateMachine<T> {

    private String name;
    protected final CustomTimer timer;

    private T currentState;
    private boolean firstStep;

    public final T getState() {
        return currentState;
    }

    public final void queueState(T nextState) {
        if (currentState == null || !currentState.equals(nextState)) {
            currentState = nextState;
            timer.reset();
            firstStep = true;
        } else {
            firstStep = false;
        }
    }

    public final boolean stateInit() {
        if (firstStep) {
            firstStep = false;
            return true;
        }
        return false;
    }

    public final boolean isState(T state) {
        return currentState.equals(state);
    }

    /** Creates a new StateMachineSubsystem. */
    public StateMachineSubsystemBase(String name) {
        this.name = name;
        firstStep = true;
        timer = new CustomTimer();
    }

    protected void inputPeriodic() {}

    protected abstract void outputPeriodic();

    public final void periodic() {
        int id = PerfTracker.start(name);
        inputPeriodic();
        handleStateMachine();
        outputPeriodic();
        PerfTracker.end(id);
        Logger.recordOutput(name + "/State", currentState);
        Logger.recordOutput(name + "/StateTimer", timer.time());
    }
}
