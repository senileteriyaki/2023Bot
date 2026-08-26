package frc.robot.subsystems.arm;

import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.devices.motor.MotorConfig;
import frc.robot.devices.motor.MotorConfig.GravityType;
import frc.robot.generated.TunerConstants;

public class ArmConstants {

    public static final MotorConfig config = new MotorConfig(8)
        .withCanbus(TunerConstants.kCANBus.getName())
        .withInverted(true)
        .withFollower(9, MotorAlignmentValue.Opposed)
        .withBrake(true)
        .withSupplyCurrentLimit(40.0)
        .withFFGains(0.0, 0.0, 0.0, 0.0)
        .withPIDGains(0.0, 0.0, 0.0, GravityType.ARM)
        .withMotionMagic(0.0, 0.0, 0.0)
        .withSim(DCMotor.getKrakenX60Foc(2), 1, 0.01);

    public static final double MIN_ANGLE_deg = 0.0;
    public static final double MAX_ANGLE_deg = 180.0;
    public static final double tol = 2.0;
    public static final double HOMING_VOLTS = -0.5;
    public static final double HOME_POSITION_deg = 0;

}
