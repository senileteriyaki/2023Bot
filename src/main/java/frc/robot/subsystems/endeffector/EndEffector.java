package frc.robot.subsystems.endeffector;

import frc.robot.devices.motor.Motor;
import frc.robot.subsystems.SubsystemBase;

public class EndEffector extends SubsystemBase<EndEffector.Command> {
    private static EndEffector instance;

    private final Motor motor;
    
    public EndEffector() {
        super("EndEffector");
        motor = new Motor("EndEffectorMotor", EEConstants.config);
        setCommand(Command.IDLE);
    }

    public enum Command {
        DISABLED,
        IDLE,
        INTAKE_CONE,
        OUTTAKE_CONE,
        INTAKE_CUBE,
        OUTTAKE_CUBE
    }

    @Override
    public void handle(){
        switch (getCommand()){
            case DISABLED:
                motor.stop();
                break;
            case IDLE:
                motor.setVoltage(0);
                break;
            case INTAKE_CONE:
                motor.setVoltage(12);
                break;
            case OUTTAKE_CONE:
                motor.setVoltage(-12);
                break;
            case INTAKE_CUBE:
                motor.setVoltage(12);
                break;
            case OUTTAKE_CUBE:
                motor.setVoltage(-12);
                break;
            default:
                break;
        }
    }

    @Override
    public void outputPeriodic(){}

    @Override
    public void inputPeriodic(){}
     

    public static EndEffector getInstance() {
        if (instance == null) {
            instance = new EndEffector();
            System.out.println("initialized EndEffector");        
        }
        return instance;
    }

}
