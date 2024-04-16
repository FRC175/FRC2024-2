package frc.robot.commands.AutoModes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.utils.Utils;
import frc.robot.Constants.ArmPosition;
import frc.robot.commands.ArmViaLimelight;
import frc.robot.commands.SetArmPosition;
import frc.robot.commands.BootlegAuto.DriveWhileIntaking;
import frc.robot.commands.Drive.LockViaLimelight;
import frc.robot.commands.Drive.ResetGyro;
import frc.robot.commands.Drive.SwerveToDist;
import frc.robot.commands.Shooter.RevShooterAuto;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;

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

     public DoubleNote(Drive drive, Shooter shooter, Intake intake, Arm arm, Limelight limelight) {
          addCommands(
               new ParallelCommandGroup(
                    new SetArmPosition(arm, 0.4, 0.6, ArmPosition.SPEAKER),
                    new SequentialCommandGroup(
                         new ResetGyro(drive, 0),
                         new RevShooterAuto(shooter, intake, 1.1),
                         new DriveWhileIntaking(intake, drive, arm, 0.3, 1.4, 0, 0),
                         // new SwerveToDist(drive, 0.4, Utils.convToSideAngle(225), Utils.convToSideAngle(342), 0.7),
                
                         new LockViaLimelight(drive, limelight, 0.5),
                         new ArmViaLimelight(limelight, arm, 1),
                         new RevShooterAuto(shooter, intake, 1)
                         
                    )
               )
          );
     }
}