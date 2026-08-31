// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Robot;
import frc.robot.devices.motor.Motor;
import frc.robot.subsystems.SubsystemBase;

public class Elevator extends SubsystemBase<Elevator.Command> {
   private static Elevator instance;

   private final Elevator2d setpoint2d = new Elevator2d("Elevator/Setpoint2d", Color.kAquamarine);
   private final Elevator2d measured2d = new Elevator2d("Elevator/Measured2d", Color.kIndianRed);

   public enum Command {
      DISABLED,
      IDLE,
      HOMING,
      MOVING, 
      MANUAL
   }

   private enum Homing {
      SEEKING,
      SETTLED
   }

   private enum TRAVEL {
      MOVING,
      HOLDING
   }

   private final Motor motor;
   private double target_m = ElevatorConstants.HOME_POSITION_m;
   private double target_v = 0.0;
   private boolean zeroed = false;

   public static Elevator getInstance() {
      // FIXED: Checks if null to initialize, rather than if not null.
      if (instance == null) {
         instance = new Elevator();
         System.out.println("initialized Elevator");
      }
      return instance;
   }

   public Elevator() {
      super("Elevator");
      motor = new Motor("ElevatorMotor", ElevatorConstants.config);
      setCommand(Command.IDLE);
   }

   @Override
   public void inputPeriodic() {
      motor.readInputs();
   }

   @Override
   public void outputPeriodic() {
      Logger.recordOutput("Elevator/Pos_m", motor.getPosition());
      Logger.recordOutput("Elevator/Velocity_mps", motor.getVelocity());
      Logger.recordOutput("Elevator/Target_m", target_m);
      
      
      setpoint2d.setHeight(target_m); 
      measured2d.setHeight(motor.getPosition());
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
                  motor.setVoltage(ElevatorConstants.HOMING_VOLTS);
                  if (Robot.isSimulation()) {
                     motor.zeroPosition(ElevatorConstants.HOME_POSITION_m);
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
            if (firstLoop()) {
               setSubstate(TRAVEL.MOVING);
            }

            switch ((TRAVEL) getSubstate()) {
               case MOVING:
                  motor.setMotionMagic(target_m);
                  if (atTarget()) {
                     setSubstate(TRAVEL.HOLDING);
                  }
                  break;
               case HOLDING:
                  motor.setMotionMagic(target_m);
                  if (!atTarget()) {
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

   public void setTarget(double meters) {
      target_m = meters;
   }

   public double getTarget() {
      return target_m;
   }

   public boolean atTarget(double tol) {
      return (Math.abs(target_m - motor.getPosition()) < tol);
   }

   public boolean atTarget() {
      return atTarget(ElevatorConstants.tol);
   }

   public void setManualVoltage(double volts) {
      target_v = volts;
      setCommand(Command.MANUAL);
   }

   public void trackToHeight(double height){
      setTarget(height);
      setCommand(Command.MOVING);
   }

   public void home(){
      setCommand(Command.HOMING);
   }
}