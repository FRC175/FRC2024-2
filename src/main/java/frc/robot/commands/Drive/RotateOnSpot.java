package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Drive.Drive;

public class RotateOnSpot extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Drive drive;

    private double headingAngle;
    private Limelight limelight = null;

    public RotateOnSpot(Drive drive, double headingAngle) {
        this.drive = drive;
        this.headingAngle = headingAngle;
        
        addRequirements(drive);
    }

    public RotateOnSpot(Drive drive, Limelight limelight) {
        this.limelight = limelight;
        this.drive = drive;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.resetDistance();

        if (limelight != null) {
            Double angle = limelight.getHorizontalAngle(11);
            this.headingAngle = angle == null ? 0 : drive.getYaw() + angle;
        }
    }


    @Override
    public void execute() {
        drive.lockSwerve(0, 0, headingAngle, drive.getYaw());
    }

    @Override
    public void end(boolean interrupted) {
        drive.setOpenLoop(0, 0);
        drive.resetDistance();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(drive.getYaw() - headingAngle) < 3;
    } 
}
