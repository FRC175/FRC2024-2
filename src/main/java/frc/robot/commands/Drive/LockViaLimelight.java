package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
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
    private boolean auto = false;
    private double time = 0;
    private double startTime;

    public LockViaLimelight(Controller joy, Drive drive, Limelight limelight) {
        this.joy = joy;
        this.drive = drive;
        this.limelight = limelight;

        addRequirements(drive);
    }

    public LockViaLimelight(Drive drive, Limelight limelight, double time) {
        this.drive = drive;
        this.limelight = limelight;
        this.joy = new Controller(new Joystick(3));
        this.auto = true;
        this.time = time;
        // this.startTime = Timer.getFPGATimestamp();

    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    public double calcAngleDisplacement() {
        int id = DriverStation.getAlliance().get() == Alliance.Red ? 4 : 7;
        double angle = limelight.getHorizontalAngle(id);
        double distance = limelight.getDistance(id);
        double shift = 6;
        double new_distance = Math.sqrt(Math.pow(distance*Math.sin(Math.toRadians(angle)) + shift, 2) + Math.pow(distance*Math.cos(Math.toRadians(angle)),2));
        double new_angle = Math.toDegrees(Math.atan2(distance*Math.sin(Math.toRadians(angle)) + shift, distance*Math.cos(Math.toRadians(angle))));
        double output = 90 - Math.toDegrees(Math.atan2(new_distance * Math.cos(Math.toRadians(new_angle)), new_distance * Math.sin(Math.toRadians(new_angle)) - 8.5));
        SmartDashboard.putNumber("Ideal Angle", output);
        return output;
        
    }

    @Override
    public void execute() {
        lockAngle = drive.getYaw() - calcAngleDisplacement();
        if (!auto)
        drive.lockSwerve(
            Utils.deadband(joy.getLeftX(), 0.1, 1), 
            Utils.deadband(joy.getLeftY(), 0.1, 1), 
            lockAngle, 
            drive.getYaw());
        else drive.lockSwerve(
            0, 
            0, 
            lockAngle, 
            drive.getYaw());
        
    }

    @Override
    public void end(boolean interrupted) {
        if (auto) {
            drive.setOpenLoop(0, 0);
        }
    }

    @Override
    public boolean isFinished() {
        if (!auto) {
            return false;
        }
        else {
            System.out.println(Timer.getFPGATimestamp() - startTime);
            return (Timer.getFPGATimestamp() - startTime > time);
        }
    }
}
