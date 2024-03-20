package frc.robot.commands;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class pickup extends Command {
    
    private final Intake intake; 
    private boolean initial;
    private boolean auto;
    private boolean phase2;
    private double startTime;
    private double goalTime;
   
    

    public pickup(Intake intake, boolean auto, double time) {
        
        this.intake = intake;
        this.auto = auto;
        initial = true;
        phase2 = false;
        goalTime = time;
       
  
        addRequirements(intake);
    }

    public pickup(Intake intake, boolean auto) {
        
        this(intake, auto, Double.MAX_VALUE);
    }

    @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    if (!auto || auto) {
      if (initial) intake.setOpenLoop(0.8);
      if (initial) if (intake.isNoteHeld()) {initial = false; phase2 = true;}
      if (phase2) intake.setOpenLoop(0.4);
      if (phase2) if (intake.isNotePresent()) phase2 = false;
      if (!initial && !phase2) intake.setOpenLoop(-0.25);
    } else {
      intake.setOpenLoop(0.5);
    }

    
  }
  
  @Override
  public void end(boolean interrupted) {
    intake.setOpenLoop(0);
    initial = true;
  }

  @Override
  public boolean isFinished() {
    System.out.println(Timer.getFPGATimestamp() - startTime);
    System.out.println(goalTime);
    if (!auto) return !initial && !phase2 && !intake.isNotePresent();
    else return (!initial && !phase2 && !intake.isNotePresent()) || (Timer.getFPGATimestamp() - startTime > goalTime);
  }
}

