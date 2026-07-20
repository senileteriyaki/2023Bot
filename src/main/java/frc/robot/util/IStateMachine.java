package frc.robot.util;

public interface IStateMachine<T extends Enum<T> & IState> {

    public abstract T getState();

    public abstract void queueState(T nextState);

    public abstract boolean stateInit();

    public abstract boolean isState(T state);

    public abstract void handleStateMachine();
}
