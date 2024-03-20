package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RevShooterAuto extends SequentialCommandGroup{
    public RevShooterAuto(Shooter shooter, Intake intake, double time) {
        addCommands(
            new RevShooterWithTimeout(shooter, 5500, 4200, time),
            new InstantCommand(() -> intake.setOpenLoop(0.7), intake),
            new WaitCommand(0.5),
            new InstantCommand(() -> {
                intake.setOpenLoop(0);
                shooter.shooterSetOpenLoop(0, 0);
            },shooter, intake));
    }
    
}
