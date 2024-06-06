package frc.robot.commands.AutoModes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmPosition;
import frc.robot.commands.ArmViaLimelight;
import frc.robot.commands.SetArmPosition;
import frc.robot.commands.BootlegAuto.DriveWhileIntaking;
import frc.robot.commands.Drive.LockViaLimelight;
import frc.robot.commands.Drive.ResetGyro;
import frc.robot.commands.Drive.RotateOnSpot;
import frc.robot.commands.Drive.SwerveToDist;
import frc.robot.commands.Shooter.RevShooterAuto;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.utils.Utils;

//start 60,
    //drive 25
public class FarDoubleSideNote extends SequentialCommandGroup{
    public FarDoubleSideNote(Drive drive, Shooter shooter, Intake intake, Arm arm, Limelight limelight) {
    // double angle = Utils.convToSideAngle(0);
        addCommands(
            new ParallelCommandGroup(
                new SetArmPosition(arm, 0.4, 0.6, ArmPosition.WHATEVER),
                new SequentialCommandGroup(
                    new ResetGyro(drive, Utils.convToSideAngle(60)),
                    // new WaitCommand(2),
                    new InstantCommand(() -> arm.setArmGoalPosition(ArmPosition.WHATEVER)),
                    // new SetArmPosition(arm, 0.4, 0.6, ArmPosition.WHATEVER),
                    // new LockViaLimelight(drive, limelight, 0.5),
                    // new ArmViaLimelight(limelight, arm, 1),
                    new RevShooterAuto(shooter, intake, 1),
                    // new InstantCommand(() -> arm.setArmGoalPosition(ArmPosition.INTAKE)),
                    new SwerveToDist(drive, 0.8, Utils.convToSideAngle(45), Utils.convToSideAngle(45), 3.90),

                    new DriveWhileIntaking(intake, drive, arm, 0.5, 5.0, Utils.convToSideAngle(5), Utils.convToSideAngle(10), 5),
                    new SwerveToDist(drive, 0.6, Utils.convToSideAngle(200), Utils.convToSideAngle(25), 6.4),
                    // new SwerveToDist(drive, 0.4, Utils.convToSideAngle(200), Utils.convToSideAngle(30), 0.6),

                    new LockViaLimelight(drive, limelight, 2),
                    new ArmViaLimelight(limelight, arm, 1),
                    new RevShooterAuto(shooter, intake, 1)

                    // new SwerveToDist(drive, 0.4, Utils.convToSideAngle(200), Utils.convToSideAngle(20), 0.6),
                    // new RotateOnSpot(drive, 60),
                    // new InstantCommand(() -> arm.setArmGoalPosition(ArmPosition.START_NOTE)),
                    // new RevShooterAuto(shooter, intake, 1),
                    // new InstantCommand(() -> arm.setArmGoalPosition(ArmPosition.INTAKE)),
                    // new SwerveToDist(drive, 0.8, Utils.convToSideAngle(45), 0, 0.3),
                    // new SwerveToDist(drive, 0.8, 0, 0, 6)
                    // new DriveWhileIntaking(intake, drive, arm, 0.5, 1, Utils.convToSideAngle(60), Utils.convToSideAngle(60))
                )
            )
    
        ); 
    }   
}
