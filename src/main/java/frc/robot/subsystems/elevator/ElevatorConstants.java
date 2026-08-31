// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.devices.motor.MotorConfig;
import frc.robot.devices.motor.MotorConfig.GravityType;
import frc.robot.generated.TunerConstants;

public class ElevatorConstants {
    public static final double HOME_POSITION_m = 0.0;
    public static final double HOMING_VOLTS = -0.5;
    public static final double tol = 0.02;

    public static final MotorConfig config = new MotorConfig(10)
            .withCanbus(TunerConstants.kCANBus.getName())
            .withInverted(true)
            .withFollower(11, MotorAlignmentValue.Opposed)
            .withBrake(true)
            .withSupplyCurrentLimit(40.0)
            .withFFGains(0.2, 3.0, 0.05, 0.9)
            .withPIDGains(60.0, 0.0, 2.0, GravityType.ELEVATOR)
            .withMotionMagic(2.0, 6.0, 60.0)
            .withSim(DCMotor.getKrakenX60Foc(2), 12.0, 10.0);
}