package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Drive.Drive;

public class LimelightToDist extends SequentialCommandGroup {

    public LimelightToDist(Drive drive, Limelight limelight) {
        addCommands(
            new RotateOnSpot(drive, limelight),
            new SwerveToDist(drive, 0.3, limelight)
        );
    }
}
