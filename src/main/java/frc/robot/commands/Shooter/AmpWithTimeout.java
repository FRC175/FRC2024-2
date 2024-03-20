package frc.robot.commands.Shooter;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class AmpWithTimeout extends Command {
    
    private final Shooter shooter;
    private final Intake intake;
   
    private double goalTime;
    private double startTime;
    private double speed;

    public AmpWithTimeout(Shooter shooter, Intake intake, double goalTime, double speed) {
        this.shooter = shooter;
        this.intake = intake;
        this.goalTime = goalTime;
        this.speed = speed;
  
        addRequirements(shooter, intake);
    }

    @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
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

  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - startTime > goalTime;
  }
}

