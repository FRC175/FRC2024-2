package frc.robot.commands.LED;

import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.LEDColor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class BlinkCycle extends Command {
    
    private final LED led;
   
    private double startTime;
    private LEDColor[] colors;
    private double length;
    private double subTime;
    private int index;

    public BlinkCycle(double length, double frequency, LEDColor[] colors) {
        this.led = LED.getInstance();
        this.colors = colors;
        this.length = length;
        this.subTime = length / frequency;
        this.index = 0;
        
        addRequirements(led);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        
    }

    @Override
    public void execute() {
        if ((Timer.getFPGATimestamp() - startTime) > subTime) {
            startTime = Timer.getFPGATimestamp();
            index = (index + 1) % colors.length;

        }

        led.setColor(colors[index]);

    }
  
    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime > length;
    }
}

