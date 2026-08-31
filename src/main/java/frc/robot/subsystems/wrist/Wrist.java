package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Robot;
import frc.robot.devices.motor.Motor;
import frc.robot.devices.motor.MotorConfig;
import frc.robot.subsystems.SubsystemBase;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.Wrist2d;
import frc.robot.subsystems.wrist.WristConstants;

public class Wrist extends SubsystemBase<Wrist.Command> {
private static Wrist instance;

   private final Wrist2d setpoint2d = new Wrist2d("Wrist/Setpoint2d", Color.kAquamarine);
   private final Wrist2d measured2d = new Wrist2d("Wrist/Measured2d", Color.kIndianRed);

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
   private double target_deg = WristConstants.HOME_POSITION_deg;
   private double target_v = 0.0;
   private boolean zeroed = false;

   public static Wrist getInstance() {
      if (instance == null) {
         instance = new Wrist();
         System.out.println("initialized Wrist");
      }
      return instance;
   }

   public Wrist() {
      super("Wrist");
      motor = new Motor("WristMotor", WristConstants.config);
      setCommand(Command.IDLE);
   }

   @Override
   public void inputPeriodic() {
      motor.readInputs();
   }

   @Override
   public void outputPeriodic() {
      Logger.recordOutput("Wrist/Pos_deg", motor.getPosition());
      Logger.recordOutput("Wrist/Velocity_dps", motor.getVelocity());
      Logger.recordOutput("Wrist/Target_deg", target_deg);
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
                  motor.setVoltage(WristConstants.HOMING_VOLTS);
                  if (Robot.isSimulation()) {
                     motor.zeroPosition(WristConstants.HOME_POSITION_deg);
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
      return atTarget(WristConstants.tol);
   }

   public void setManualVoltage(double volts){
      target_v = volts;
      setCommand(Command.MANUAL);
   }

   public void home(){
      setCommand(Command.HOMING);
   }
   
   public void trackToAngle(double angle){
      target_deg = angle;
      setCommand(Command.MOVING);
   }
}
