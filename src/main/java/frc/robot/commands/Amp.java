package frc.robot.commands;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;

public class Amp extends Command {
    
    private final Intake intake; 
    private final Shooter shooter;
    private double speed;
   
    

    public Amp(Intake intake, Shooter shooter, double speed) {
        
        this.intake = intake;
        this.speed = speed;
        this.shooter = shooter;
       
  
        addRequirements(intake, shooter);
    }

    @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    intake.setOpenLoop(speed);
    shooter.shooterSetOpenLoop(speed, speed);
  }
  
  @Override
  public void end(boolean interrupted) {
    intake.setOpenLoop(0);
    shooter.shooterSetOpenLoop(0, 0);
  }
}

