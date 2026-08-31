package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Elevator2d {
    LoggedMechanism2d mech;

    public LoggedMechanismLigament2d elev;

    String name;

    public Elevator2d(String name, Color color){
        this.name = name;
        mech = new LoggedMechanism2d(4, 4);
        elev = mech.getRoot("Root", 2, 2)
            .append(new LoggedMechanismLigament2d("Elevator", 0.5, 90, 10, new Color8Bit(color)));
        
    }

    public void setHeight(double height){
        elev.setLength(height);
    }

    public void periodic(){
        SmartDashboard.putData(name, mech);
        Logger.recordOutput(name, mech);
    }
}