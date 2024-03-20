package frc.robot.commands.LED;

import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.LEDColor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class BlinkAlliance extends Command {
    
    private final LED led;
   
    private double startTime;
    private LEDColor[] colors;
    private double frequency;
    private double length;
    private double subTime;
    private int index;

    public BlinkAlliance() {
        this.led = LED.getInstance();
        this.length = 4;
        this.frequency = 8;
        this.subTime = length / frequency;
        this.index = 0;
        
        addRequirements(led);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        
    if(DriverStation.getAlliance().get() == Alliance.Red) {
      colors = new LEDColor[] {LEDColor.YELLOW, LEDColor.RED};
    } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
      colors = new LEDColor[] {LEDColor.YELLOW, LEDColor.BLUE};
    }

    
        
        
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

