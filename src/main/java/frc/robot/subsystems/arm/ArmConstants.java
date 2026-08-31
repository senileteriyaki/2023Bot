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
            .withFFGains(0.15, 8.4, 0.1, 0.5)
            .withPIDGains(50.0, 0.0, 1.5, GravityType.ARM)
            .withMotionMagic(1.2, 3.5, 35.0)
            .withSim(DCMotor.getKrakenX60Foc(2), 70.0, 1.2);

    public static final double MIN_ANGLE_deg = -35.0;
    public static final double MAX_ANGLE_deg = 130.0;
    public static final double tol = 2.0;
    public static final double HOMING_VOLTS = -0.5;
    public static final double HOME_POSITION_deg = 0;

}