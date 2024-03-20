package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.utils.Controller;
import frc.robot.utils.Utils;

public class LockViaLimelight extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Drive drive;
    private final Limelight limelight;
    private final Controller joy;
    private double lockAngle = 0;

    public LockViaLimelight(Controller joy, Drive drive, Limelight limelight) {
        this.joy = joy;
        this.drive = drive;
        this.limelight = limelight;

        addRequirements(drive);
    }

    @Override
    public void initialize() {

    }

    public double calcAngleDisplacement() {
        int id = DriverStation.getAlliance().get() == Alliance.Red ? 4 : 7;
        double angle = limelight.getHorizontalAngle(id);
        double distance = limelight.getDistance(id);
        double shift = 5;
        double new_distance = Math.sqrt(Math.pow(distance*Math.sin(Math.toRadians(angle)) + shift, 2) + Math.pow(distance*Math.cos(Math.toRadians(angle)),2));
        double new_angle = Math.toDegrees(Math.atan2(distance*Math.sin(Math.toRadians(angle)) + shift, distance*Math.cos(Math.toRadians(angle))));
        double output = 90 - Math.toDegrees(Math.atan2(new_distance * Math.cos(Math.toRadians(new_angle)), new_distance * Math.sin(Math.toRadians(new_angle)) - 8.0));
        SmartDashboard.putNumber("Ideal Angle", output);
        return output;
        
    }

    @Override
    public void execute() {
        lockAngle = drive.getYaw() - calcAngleDisplacement();
        drive.lockSwerve(
            Utils.deadband(joy.getLeftX(), 0.1, 1), 
            Utils.deadband(joy.getLeftY(), 0.1, 1), 
            lockAngle, 
            drive.getYaw());
        
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
