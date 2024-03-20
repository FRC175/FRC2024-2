package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AmpAuto extends SequentialCommandGroup{
    public AmpAuto(Shooter shooter, Intake intake) {
        addCommands(
            new InstantCommand(() -> {
                intake.setOpenLoop(0.4);
                shooter.shooterSetOpenLoop(0.4, 0.4);
              }, intake, shooter));
             

    }
    
}