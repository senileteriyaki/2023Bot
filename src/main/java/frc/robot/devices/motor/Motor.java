package frc.robot.devices.motor;

import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Motor {

    public enum Mode {
        REAL,
        SIM
    }

    public enum ControlType {
        VOLTAGE,
        OPEN_LOOP,
        VELOCITY,
        POSITION_VOLTAGE,
        MOTION_MAGIC,
        MOTION_MAGIC_VELOCITY,
        STOP
    }

    private final String name;
    private final MotorIO io;
    private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

    private ControlType control = ControlType.STOP;
    private double target = 0.0;

    public Motor(String name, MotorConfig config, Mode mode) {
        this.name = name;
        if (mode == Mode.REAL) {
            this.io = new MotorIOTalonFX(config);
        } else {
            this.io = new MotorIOSim(config);
        }
    }

    public Motor(String name, MotorConfig config) {
        this(name, config, Constants.currentMode == Constants.Mode.REAL ? Mode.REAL : Mode.SIM);
    }

    public void readInputs() {
        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);
    }

    private void setOutput(ControlType control, double target) {
        this.control = control;
        this.target = target;
        Logger.recordOutput(name + "/ControlType", control);
        Logger.recordOutput(name + "/TargetSetpoint", target);
    }

    public void setVoltage(double volts) {
        setOutput(ControlType.VOLTAGE, volts);
        io.setVoltage(volts);
    }

    public void setOpenLoop(double dutyCycle) {
        setOutput(ControlType.OPEN_LOOP, dutyCycle);
        io.setOpenLoop(dutyCycle);
    }

    public void setVelocity(double velocity) {
        setOutput(ControlType.VELOCITY, velocity);
        io.setVelocity(velocity);
    }

    public void setPositionVoltage(double position) {
        setOutput(ControlType.POSITION_VOLTAGE, position);
        io.setPositionVoltage(position);
    }

    public void setMotionMagic(double position) {
        setOutput(ControlType.MOTION_MAGIC, position);
        io.setMotionMagic(position);
    }

    public void setMotionMagicVelocity(double velocity) {
        setOutput(ControlType.MOTION_MAGIC_VELOCITY, velocity);
        io.setMotionMagicVelocity(velocity);
    }

    public void stop() {
        setOutput(ControlType.STOP, 0.0);
        io.stop();
    }

    public void setBrakeMode(boolean brake) {
        io.setBrakeMode(brake);
    }

    public void zeroPosition(double position) {
        io.zeroPosition(position);
    }

    public double getPosition() {
        return inputs.position;
    }

    public double getVelocity() {
        return inputs.velocity;
    }

    public double getAcceleration() {
        return inputs.acceleration;
    }

    public double getVoltage() {
        return inputs.appliedVolts;
    }

    public double getCurrent() {
        return inputs.statorCurrent;
    }

    public boolean isConnected() {
        return inputs.connected;
    }

    public ControlType getControl() {
        return control;
    }

    public double getSetpoint() {
        return target;
    }
}
