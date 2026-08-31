// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Robot;
import frc.robot.devices.motor.Motor;
import frc.robot.subsystems.SubsystemBase;

/** Add your docs here. */
public class Arm extends SubsystemBase<Arm.Command> {
   private static Arm instance;

   private final Arm2d setpoint2d = new Arm2d("Arm/Setpoint2d", Color.kAquamarine);
   private final Arm2d measured2d = new Arm2d("Arm/Measured2d", Color.kIndianRed);

   public enum Command {
      DISABLED,
      IDLE,
      HOMING,
      MOVING, MANUAL
   }

   private enum Homing {
      SEEKING,
      SETTLED
   }

   private enum TRAVEL{
      MOVING,
      HOLDING
   }

   private final Motor motor;
   private double target_deg = ArmConstants.HOME_POSITION_deg;
   private double target_v = 0.0;
   private boolean zeroed = false;

   public static Arm getInstance() {
      if (instance == null) {
         instance = new Arm();
         System.out.println("initialized Arm");
      }
      return instance;
   }

   public Arm() {
      super("Arm");
      motor = new Motor("ArmMotor", ArmConstants.config);
      setCommand(Command.IDLE);
   }

   @Override
   public void inputPeriodic() {
      motor.readInputs();
   }

   @Override
   public void outputPeriodic() {
      Logger.recordOutput("Arm/Pos_deg", motor.getPosition());
      Logger.recordOutput("Arm/Velocity_dps", motor.getVelocity());
      Logger.recordOutput("Arm/Target_deg", target_deg);
      setpoint2d.setAngle(target_deg);
      measured2d.setAngle(motor.getPosition());
      measured2d.periodic();
      setpoint2d.periodic();
   }

   @Override
   public void handle() {
      switch (getCommand()) {
         case DISABLED:
            motor.stop();
            break;
         case IDLE:
            motor.setVoltage(0.0);
            break;
         case HOMING:
            if (firstLoop()) {
               setSubstate(zeroed ? Homing.SETTLED : Homing.SEEKING);
            }
            switch ((Homing) getSubstate()) {
               case SEEKING:
                  motor.setVoltage(ArmConstants.HOMING_VOLTS);
                  if (Robot.isSimulation()) {
                     motor.zeroPosition(ArmConstants.HOME_POSITION_deg);
                     zeroed = true;
                     setSubstate(Homing.SETTLED);
                  }
                  break;
               case SETTLED:
                  setCommand(Command.IDLE);
                  break;

            }
            break;
         case MOVING:
            if (firstLoop()){
               setSubstate(TRAVEL.MOVING);
               
            }

            switch ((TRAVEL) getSubstate()){
               case MOVING:
                  motor.setMotionMagic(target_deg);
                  if (atTarget()){
                     setSubstate(TRAVEL.HOLDING);
                  }
                  break;
               case HOLDING:
                  motor.setMotionMagic(target_deg);
                  if (!atTarget()){
                     setSubstate(TRAVEL.MOVING);
                  }
                  break;
            }
            break;

         case MANUAL:
            motor.setVoltage(target_v);
            break;

      }
   }

   public void setTarget(double angle) {
      target_deg = angle;
   }

   public double getTarget() {
      return target_deg;
   }

   public boolean atTarget(double tol) {
      return (Math.abs(target_deg - motor.getPosition()) < tol);
   }

   public boolean atTarget() {
      return atTarget(ArmConstants.tol);
   }

   public void setManualVoltage(double volts){
      target_v = volts;
      setCommand(Command.MANUAL);
   }

   public void trackToAngle(double angle){
      setTarget(angle);
      setCommand(Command.MOVING);
   }

   public void home(){
      setCommand(Command.HOMING);
   }

}
