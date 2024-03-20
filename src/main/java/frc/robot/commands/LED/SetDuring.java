package frc.robot.commands.LED;

import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.LEDColor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
/*  CHARLIE TO DO LIST
 - make led default jimmy blink yellow black
 - bbut not black another color
 maybe red/blue depending on alliance?
 - when note gotten make orange
 - shoot color
 - back to default
 */
public class SetDuring extends Command {
    
    private final LED led;
   
    private double startTime;
    private LEDColor color;
    private double length;


    public SetDuring(double length, LEDColor color) {
        this.led = LED.getInstance();
        this.color = color;
        this.length = length;
        
        addRequirements(led);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        
    }

    @Override
    public void execute() {
        led.setColor(color);

    }
  
    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime > length;
    }
}

