package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Wrist2d {
    LoggedMechanism2d mech;

    public LoggedMechanismLigament2d wrist;

    String name;

    public Wrist2d(String name, Color color){
        this.name = name;
        mech = new LoggedMechanism2d(4, 4);
        wrist = mech.getRoot("Root", 2, 2)
            .append(new LoggedMechanismLigament2d("Wrist", 0.5, 0, 10, new Color8Bit(color)));
        
    }

    public void setAngle(double angle){
        wrist.setAngle(angle);
    }

    public void periodic(){
        SmartDashboard.putData(name, mech);
        Logger.recordOutput(name, mech);
    }
}
