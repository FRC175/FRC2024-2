package frc.robot.commands.AutoModes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmPosition;
import frc.robot.commands.SetArmPosition;
import frc.robot.commands.BootlegAuto.DriveWhileIntaking;
import frc.robot.commands.Drive.ResetGyro;
import frc.robot.commands.Drive.SwerveToDist;
import frc.robot.commands.Shooter.RevShooterAuto;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.utils.Utils;

public class MultiNote extends SequentialCommandGroup {

   public MultiNote(Drive drive, Shooter shooter, Intake intake, Arm arm) {
    // double angle = Utils.convToSideAngle(0);
    addCommands(
        new ParallelCommandGroup(
            new SetArmPosition(arm, 0.4, 0.6, ArmPosition.SPEAKER),
            new SequentialCommandGroup(
                new ResetGyro(drive, 0),
                new RevShooterAuto(shooter, intake, 1.1),
                new DriveWhileIntaking(intake, drive, arm, 0.3, 1.4, 0, 0),
                new SwerveToDist(drive, 0.4, Utils.convToSideAngle(225), Utils.convToSideAngle(342), 0.7),
                
                new InstantCommand(() -> arm.setArmGoalPosition(ArmPosition.START_NOTE)),
                new RevShooterAuto(shooter, intake, 1),
                new DriveWhileIntaking(intake, drive, arm, 0.1, 0.8, Utils.convToSideAngle(330), Utils.convToSideAngle(300)),
                new SwerveToDist(drive, 0.4, Utils.convToSideAngle(120), 0, 1.7),
                new InstantCommand(() -> arm.setArmGoalPosition(ArmPosition.SPEAKER)),
                new RevShooterAuto(shooter, intake, 1),
                new DriveWhileIntaking(intake, drive, arm, 0.4, 1.4, Utils.convToSideAngle(35), Utils.convToSideAngle(35)),
                new InstantCommand(() -> arm.setArmGoalPosition(ArmPosition.WHATEVER)),
                new SwerveToDist(drive, 0.4, Utils.convToSideAngle(205), Utils.convToSideAngle(25), 0.6),
                new RevShooterAuto(shooter, intake, 1)
            )
        )
    
    ); 
}
}