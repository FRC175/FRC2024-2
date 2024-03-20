package frc.robot.commands.AutoModes;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.utils.Utils;
import frc.robot.Constants.ArmPosition;
import frc.robot.commands.SetArmPosition;
import frc.robot.commands.Drive.ResetGyro;
import frc.robot.commands.Drive.SwerveToDist;
import frc.robot.commands.Shooter.RevShooterAuto;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class DoubleSideShootFromSubwoofer extends SequentialCommandGroup {

   public DoubleSideShootFromSubwoofer(Drive drive, Shooter shooter, Intake intake, Arm arm) {
    double angle = Utils.convToSideAngle(54);
    addCommands(
        new ParallelCommandGroup(
            new SetArmPosition(arm, 0.2, 0.2, ArmPosition.SPEAKER),
            new SequentialCommandGroup(
                new ResetGyro(drive, angle),
                new RevShooterAuto(shooter, intake, 2),
                new SwerveToDist(drive, 0.2, angle, angle,4)
            )
        )
    
    ); 
}
}