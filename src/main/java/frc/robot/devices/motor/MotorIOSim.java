package frc.robot.devices.motor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import frc.robot.Constants;

public class MotorIOSim implements MotorIO {

    private enum Mode {
        VOLTAGE,
        OPEN_LOOP,
        POSITION,
        VELOCITY,
        NEUTRAL
    }

    private final MotorConfig config;
    private final DCMotorSim sim;

    private Mode mode = Mode.NEUTRAL;
    private double setpoint = 0.0;
    private double appliedVolts = 0.0;

    public MotorIOSim(MotorConfig config) {
        this.config = config;
        this.sim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(config.simMotor, config.simMOI, config.simGearing),
                config.simMotor);
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        double pos = sim.getAngularPositionRotations();
        double vel = sim.getAngularVelocityRPM() / 60.0;

        appliedVolts = MathUtil.clamp(computeVolts(pos, vel), -12.0, 12.0);
        sim.setInputVoltage(appliedVolts);
        sim.update(Constants.globalDelta_s);

        inputs.connected = true;
        inputs.position = sim.getAngularPositionRotations();
        inputs.velocity = sim.getAngularVelocityRPM() / 60.0;
        inputs.acceleration = sim.getAngularAccelerationRadPerSecSq() / (2.0 * Math.PI);
        inputs.appliedVolts = appliedVolts;
        inputs.supplyCurrent = sim.getCurrentDrawAmps();
        inputs.statorCurrent = sim.getCurrentDrawAmps();
        inputs.tempC = 25.0;
    }

    private double computeVolts(double pos, double vel) {
        switch (mode) {
            case VOLTAGE:
                return setpoint;
            case OPEN_LOOP:
                return setpoint * 12.0;
            case POSITION: {
                double error = setpoint - pos;
                return gravityFeedforward()
                        + config.kP * error
                        - config.kD * vel
                        + Math.signum(error) * config.kS;
            }
            case VELOCITY: {
                double error = setpoint - vel;
                return gravityFeedforward()
                        + config.kV * setpoint
                        + 0.5 * error
                        + Math.signum(setpoint) * config.kS;
            }
            case NEUTRAL:
                return 0.0;
            default:
                return 0.0;
        }
    }

    private double gravityFeedforward() {
        return config.gravity == MotorConfig.GravityType.NONE ? 0.0 : config.kG;
    }

    @Override
    public void setVoltage(double volts) {
        mode = Mode.VOLTAGE;
        setpoint = volts;
    }

    @Override
    public void setOpenLoop(double dutyCycle) {
        mode = Mode.OPEN_LOOP;
        setpoint = dutyCycle;
    }

    @Override
    public void setVelocity(double velocity) {
        mode = Mode.VELOCITY;
        setpoint = velocity;
    }

    @Override
    public void setPositionVoltage(double position) {
        mode = Mode.POSITION;
        setpoint = position;
    }

    @Override
    public void setMotionMagic(double position) {
        mode = Mode.POSITION;
        setpoint = position;
    }

    @Override
    public void setMotionMagicVelocity(double velocity) {
        mode = Mode.VELOCITY;
        setpoint = velocity;
    }

    @Override
    public void stop() {
        mode = Mode.NEUTRAL;
        setpoint = 0.0;
    }

    @Override
    public void zeroPosition(double position) {
        sim.setState(position * 2.0 * Math.PI, 0.0);
    }
}
