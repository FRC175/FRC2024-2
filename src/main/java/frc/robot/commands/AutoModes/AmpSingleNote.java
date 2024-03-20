package frc.robot.commands.AutoModes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import frc.robot.commands.SetArmPosition;
import frc.robot.commands.BootlegAuto.DriveWhileIntaking;
import frc.robot.commands.Drive.ResetGyro;
import frc.robot.commands.Drive.RotateOnSpot;
import frc.robot.commands.Drive.SwerveToDist;
import frc.robot.commands.Shooter.AmpWithTimeout;
import frc.robot.commands.Shooter.RevShooterAuto;
import frc.robot.commands.Shooter.RevShooterWithTimeout;
import frc.robot.utils.Utils;
import frc.robot.Constants.ArmPosition;

public class AmpSingleNote extends SequentialCommandGroup {
    
    public AmpSingleNote(Drive drive, Shooter shooter, Intake intake, Arm arm) {
     addCommands(
        new ParallelCommandGroup(
            new SetArmPosition(arm, 0.4, 0.6, ArmPosition.AMP),
            new SequentialCommandGroup(
                new ResetGyro(drive, Utils.convToSideAngle(90)),
                new InstantCommand(() -> arm.setArmGoalPosition(ArmPosition.AMP)),
                new SwerveToDist(drive, 0.3, Utils.convToSideAngle(270), Utils.convToSideAngle(90), 0.5),
                new SwerveToDist(drive, 0.3, 0, Utils.convToSideAngle(90), 0.4),
                new WaitCommand(1),
                new AmpWithTimeout(shooter, intake, 0.5, 0.4),
                new InstantCommand(() -> arm.setArmGoalPosition(ArmPosition.INTAKE)),
                new SwerveToDist(drive, 0.7, 0, Utils.convToSideAngle(90), 1.8),
                new SwerveToDist(drive, 0.7, 0, 0, 4.2),
                new DriveWhileIntaking(intake, drive, arm, 0.2, 1, Utils.convToSideAngle(15), Utils.convToSideAngle(15)),
                new SwerveToDist(drive, 0.4, Utils.convToSideAngle(178), 0, 6),
                new RotateOnSpot(drive, Utils.convToSideAngle(305)),
                new InstantCommand(() -> arm.setArmGoalPosition(ArmPosition.MARRR)),
                new RevShooterAuto(shooter, intake, 1)
                

        )));
    }
}
