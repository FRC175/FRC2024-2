package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shuffleboard;
import frc.robot.subsystems.Drive.Drive;

public class SwerveToAngle extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Drive drive;
    private final Shuffleboard shuffleboard;

    private final double speed;
    private final double dist;
    private double angle = 0;
    private double angle2 = 0;
    


    public SwerveToAngle(Drive drive, Shuffleboard shuffleboard, double dist, double speed) {
    
        this.drive = drive;
        this.shuffleboard = shuffleboard;
        this.dist = dist;
        this.speed = speed;

        
        addRequirements(drive, shuffleboard);
    }

    @Override
    public void initialize() {
        drive.resetDistance();
        angle = (drive.getYaw() - shuffleboard.limelight.getHorizontalAngle(11) + 360) % 360;
        angle2 = (drive.getYaw() - shuffleboard.limelight.getHorizontalAngle(11) + 360 + 180) % 360;
    }


    @Override
    public void execute() {
        
        double joyX = speed * Math.cos(Math.toRadians((angle2+90) % 360));
        double joyY = -speed * Math.sin(Math.toRadians((angle2+90) % 360));
        drive.lockSwerve(joyX, joyY, angle, drive.getYaw());
        SmartDashboard.putNumber("Goal Distance", drive.getDriveDistance());
    }

    @Override
    public void end(boolean interrupted) {
        drive.setOpenLoop(0, 0);
        drive.resetDistance();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(drive.getDriveDistance()) >= dist;
    } 
}
