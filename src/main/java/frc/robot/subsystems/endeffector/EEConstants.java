package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.devices.motor.MotorConfig;
import frc.robot.devices.motor.MotorConfig.GravityType;
import frc.robot.generated.TunerConstants;

public class EEConstants {
    public static final MotorConfig config = new MotorConfig(15)
        .withCanbus(TunerConstants.kCANBus.getName())
        .withInverted(true)
        .withFollower(16, MotorAlignmentValue.Opposed)
        .withBrake(true)
        .withSupplyCurrentLimit(40.0)
        .withFFGains(0.0, 0.0, 0.0, 0.0)
        .withPIDGains(0.0, 0.0, 0.0, GravityType.ARM)
        .withMotionMagic(0.0, 0.0, 0.0)
        .withSim(DCMotor.getKrakenX60Foc(1), 1, 0.01);
}
