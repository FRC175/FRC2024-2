package frc.robot.commands.AutoModes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.Constants.ArmPosition;
import frc.robot.commands.SetArmPosition;
import frc.robot.commands.BootlegAuto.DriveWhileIntaking;
import frc.robot.commands.Drive.SwerveToDist;
import frc.robot.commands.Shooter.RevShooterAuto;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class DoubleNote extends SequentialCommandGroup {

     // public DoubleNote(Drive drive, Shooter shooter, Intake intake, Arm arm) {
     //      addCommands(
     //           new RunToPositionAndShoot(arm, shooter, intake, ArmPosition.SPEAKER),
     //           new SetArmPosition(arm, 0.1, 0.2, false, ArmPosition.INTAKE),
     //           new ParallelCommandGroup(
     //                new pickup(intake), 
     //                new SwerveToDist(drive, 0.2, 0, 0, 1)
     //           ),
     //           new SwerveToDist(drive, 0, 180, 0, 1),
     //           new RunToPositionAndShoot(arm, shooter, intake, ArmPosition.SPEAKER)
     //      );
     // }

     public DoubleNote(Drive drive, Shooter shooter, Intake intake, Arm arm) {
          addCommands(
               new ParallelCommandGroup(
                    new SetArmPosition(arm, 0.3, 0.3, ArmPosition.SPEAKER),
                    new SequentialCommandGroup(
                         new RevShooterAuto(shooter, intake, 2),
                         new DriveWhileIntaking(intake, drive, arm, 0.2, 2, 0, 0),
                         new SwerveToDist(drive, 0.2, 180, 0, 2),
                         new InstantCommand(() -> {arm.setArmGoalPosition(ArmPosition.START_NOTE);}),
                         new RevShooterAuto(shooter, intake, 2)
                    )
               )
          );
     }
}