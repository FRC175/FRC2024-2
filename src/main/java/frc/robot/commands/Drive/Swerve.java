package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.utils.Controller;
import frc.robot.utils.Utils;

public class Swerve extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Drive drive;
    private final Controller joy;

    public Swerve(Controller joy, Drive drive) {
        this.joy = joy;
        this.drive = drive;

        addRequirements(drive);
    }

    @Override
    public void initialize() {

    }


    @Override
    public void execute() {
        double twistDeadband = 0.35;
        double twist = Utils.deadband(joy.getTwist(), twistDeadband) * -1;
        if (twist < 0) twist = -1 + (1/(1-twistDeadband)) * (twist + 1);
        else if (twist > 0) twist = 1 + (1/(1-twistDeadband)) * (twist - 1);
        drive.swerve(
            Utils.deadband(joy.getLeftX(), 0.1, 1 * (1-Constants.TODDLER_MODE)), 
            Utils.deadband(joy.getLeftY(), 0.1, 1 * (1-Constants.TODDLER_MODE)), 
            Math.pow((twist * 0.85), 1), 
            drive.getYaw());
        // drive.postEncoders();
        // drive.postYaw();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
