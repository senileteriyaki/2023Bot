package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.devices.motor.MotorConfig;
import frc.robot.devices.motor.MotorConfig.GravityType;
import frc.robot.generated.TunerConstants;

public class WristConstants {
    public static final MotorConfig config = new MotorConfig(12)
        .withCanbus(TunerConstants.kCANBus.getName())
        .withInverted(true)
        .withFollower(13, MotorAlignmentValue.Opposed)
        .withBrake(true)
        .withSupplyCurrentLimit(40.0)
        .withFFGains(0.1, 4.8, 0.05, 0.2)
        .withPIDGains(35.0, 0.0, 1.0, GravityType.ARM)
        .withMotionMagic(2.0, 6.0, 60.0)
        .withSim(DCMotor.getKrakenX60Foc(2), 40.0, 0.05);

    public static final double MIN_ANGLE_deg = 0;
    public static final double MAX_ANGLE_deg = 70.0;
    public static final double tol = 2.0;
    public static final double HOMING_VOLTS = -0.5;
    public static final double HOME_POSITION_deg = 0;
}