package frc.robot.devices.motor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import frc.robot.Constants;

public class MotorIOSim2 implements MotorIO {

    private enum Mode {
        VOLTAGE,
        OPEN_LOOP,
        POSITION,
        MOTION_MAGIC,
        VELOCITY,
        NEUTRAL
    }

    private final MotorConfig config;
    private final DCMotorSim sim;

    private Mode mode = Mode.NEUTRAL;
    private double setpoint = 0.0;
    private double appliedVolts = 0.0;

    // Profile states for Motion Magic
    private TrapezoidProfile.State currentProfileState = new TrapezoidProfile.State();
    private TrapezoidProfile.State targetProfileState = new TrapezoidProfile.State();

    public MotorIOSim2(MotorConfig config) {
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
                // Standard unprofiled position control (Step Input)
                double error = setpoint - pos;
                return gravityFeedforward()
                        + config.kP * error
                        - config.kD * vel
                        + Math.signum(error) * config.kS;
            }
            case MOTION_MAGIC: {
                // Profiled position control (Uses kV based on generated velocity trajectory)
                TrapezoidProfile profile = new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(config.mmCruiseVelocity, config.mmAcceleration)
                );

                // Advance the profile by one loop iteration
                currentProfileState = profile.calculate(Constants.globalDelta_s, currentProfileState, targetProfileState);

                double posError = currentProfileState.position - pos;
                double velError = currentProfileState.velocity - vel;

                double feedforward = gravityFeedforward() 
                                   + (config.kV * currentProfileState.velocity) 
                                   + (Math.signum(currentProfileState.velocity) * config.kS);

                double feedback = (config.kP * posError) + (config.kD * velError);

                return feedforward + feedback;
            }
            case VELOCITY: {
                double error = setpoint - vel;
                return gravityFeedforward()
                        + config.kV * setpoint
                        + config.kP * error // Uses config kP instead of hardcoded 0.5
                        + Math.signum(setpoint) * config.kS;
            }
            case NEUTRAL:
                return 0.0;
            default:
                return 0.0;
        }
    }

    private double gravityFeedforward() {
        if (config.gravity == MotorConfig.GravityType.NONE){
            return 0.0;
        }

        if (config.gravity == MotorConfig.GravityType.ELEVATOR){
            return config.kG;
        }

        if (config.gravity == MotorConfig.GravityType.ARM){
            return config.kG * Math.cos(sim.getAngularPositionRotations() * 2 * Math.PI);
        }

        return config.kG;
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
        if (mode != Mode.MOTION_MAGIC) {
            // Seed the profile state with our current physical state when we first enter the mode
            // This prevents harsh jumping if activated while already moving
            currentProfileState = new TrapezoidProfile.State(
                sim.getAngularPositionRotations(), 
                sim.getAngularVelocityRPM() / 60.0
            );
        }
        mode = Mode.MOTION_MAGIC;
        targetProfileState = new TrapezoidProfile.State(position, 0.0);
    }

    @Override
    public void setMotionMagicVelocity(double velocity) {
        // Technically this acts identically to velocity mode in CTRE logic unless you use jerk/accel profiling,
        // but for now we route it to standard velocity mode
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
        // Also ensure we reset the motion magic state so it doesn't try to fly back
        currentProfileState = new TrapezoidProfile.State(position, 0.0);
    }
}