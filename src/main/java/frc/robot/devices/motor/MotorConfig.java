package frc.robot.devices.motor;

import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.system.plant.DCMotor;


public class MotorConfig {

    public enum GravityType {
        NONE,
        ELEVATOR,
        ARM
    }

    public int canId;
    public String canbus = "rio";
    public Integer followerId = null;
    public MotorAlignmentValue followerOppose = MotorAlignmentValue.Opposed;

    public boolean inverted = false;
    public boolean brake = true;
    public double supplyCurrentLimit = 40.0;
    public boolean foc = true;

    public double sensorToMechanismRatio = 1.0;

    public double kS = 0.0;
    public double kV = 0.0;
    public double kA = 0.0;
    public double kG = 0.0;
    public double kP = 0.0;
    public double kI = 0.0;
    public double kD = 0.0;
    public GravityType gravity = GravityType.NONE;

    public double mmCruiseVelocity = 0.0;
    public double mmAcceleration = 0.0;
    public double mmJerk = 0.0;

    public DCMotor simMotor = DCMotor.getKrakenX60Foc(1);
    public double simGearing = 1.0;
    public double simMOI = 0.001;

    public MotorConfig(int canId) {
        this.canId = canId;
    }

    public MotorConfig withCanbus(String canbus) {
        this.canbus = canbus;
        return this;
    }

    public MotorConfig withFollower(int followerId, MotorAlignmentValue oppose) {
        this.followerId = followerId;
        this.followerOppose = oppose;
        return this;
    }

    public MotorConfig withInverted(boolean inverted) {
        this.inverted = inverted;
        return this;
    }

    public MotorConfig withBrake(boolean brake) {
        this.brake = brake;
        return this;
    }

    public MotorConfig withSupplyCurrentLimit(double amps) {
        this.supplyCurrentLimit = amps;
        return this;
    }

    public MotorConfig withFoc(boolean foc) {
        this.foc = foc;
        return this;
    }

    public MotorConfig withSensorToMechanismRatio(double ratio) {
        this.sensorToMechanismRatio = ratio;
        return this;
    }

    public MotorConfig withFFGains(
            double kS, double kV, double kA, double kG) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kG = kG;
        return this;
    }

    public MotorConfig withPIDGains( double kP, double kI, double kD, GravityType gravity) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.gravity = gravity;
        return this;
    }

    public MotorConfig withMotionMagic(double cruiseVelocity, double acceleration, double jerk) {
        this.mmCruiseVelocity = cruiseVelocity;
        this.mmAcceleration = acceleration;
        this.mmJerk = jerk;
        return this;
    }

    public MotorConfig withSim(DCMotor simMotor, double simGearing, double simMOI) {
        this.simMotor = simMotor;
        this.simGearing = simGearing;
        this.simMOI = simMOI;
        return this;
    }
}
