package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.SubsystemBase;

public class LED extends SubsystemBase<LED.Command> {
    private static LED instance;
    private final int numLeds = 60;
    private AddressableLED leds = new AddressableLED(0);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(numLeds);

    public enum Command {
        DISABLED(
            LEDPattern.solid(new Color(0, 0, 0))),
        IDLE(
            LEDPattern.solid(Color.kAntiqueWhite)),
        INTAKE(
            LEDPattern.solid(Color.kCyan).blink(Seconds.of(0.5))),
        OUTTAKE(
            LEDPattern.solid(Color.kYellow).blink(Seconds.of(0.5))),
        CLIMBING(
            LEDPattern.solid(Color.kGreen).blink(Seconds.of(0.5))),
        SCORING(
            LEDPattern.solid(Color.kOrangeRed).blink(Seconds.of(0.5))),
        ERROR(
            LEDPattern.solid(Color.kRed));

        public final LEDPattern pattern;

        private Command(LEDPattern pattern){
            this.pattern = pattern;
        }
    }

    public LED() {
        super("LED");
        leds.setLength(numLeds);
        leds.start();
        setCommand(Command.DISABLED);
    }

    public static LED getInstance(){
        if (instance == null){
            instance = new LED();
            System.out.println("initialized LED");
        }
        return instance;
    }
    

    @Override
    public void handle() {
        getCommand().pattern.applyTo(buffer);
    }

    @Override
    public void outputPeriodic(){
        leds.setData(buffer);
    }

    @Override
    public void inputPeriodic() {}
    
}
