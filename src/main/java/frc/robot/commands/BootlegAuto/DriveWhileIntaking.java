package frc.robot.commands.BootlegAuto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmPosition;
import frc.robot.commands.pickup;
import frc.robot.commands.Drive.SwerveToDist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.utils.Utils;

public class DriveWhileIntaking extends SequentialCommandGroup {

    public DriveWhileIntaking(Intake intake, Drive drive, Arm arm, double speed, double dist, double headingAngle, double transAngle, double time) {
        addCommands(
            new ParallelCommandGroup(
                new InstantCommand(() -> {arm.setArmGoalPosition(ArmPosition.INTAKE);}),
                new pickup(intake, true, time),
                new SwerveToDist(drive, speed, transAngle, headingAngle, dist)
            )
        );

        
    }
    public DriveWhileIntaking(Intake intake, Drive drive, Arm arm, double speed, double dist, double headingAngle, double transAngle) {
            this(intake, drive, arm, speed, dist, headingAngle, transAngle, 3);
        }
    }

