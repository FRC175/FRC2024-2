package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmPosition;
import frc.robot.subsystems.Arm;

public class SetArmPosition extends Command {
    
   private final Arm arm; 
   private double downSpeed;
   private double upSpeed;
   private ArmPosition armGoal = ArmPosition.SPEAKER;
   private double armGoalPosition;
   private final double UP_DEADBAND = 0.1;
   private final double DOWN_DEADBAND = 0.05;

    public SetArmPosition(Arm arm, double downSpeed, double upSpeed, boolean isFinite) {
        this(arm, downSpeed, upSpeed, arm.getArmGoalPosition());
    }
    

    public SetArmPosition(Arm arm, double downSpeed, double upSpeed, ArmPosition armGoal) {
        this(arm, downSpeed, upSpeed, armGoal.getPosition());
        
    }

    public SetArmPosition(Arm arm, double downSpeed, double upSpeed, double goal) {
        this.arm = arm;
        this.downSpeed = downSpeed;
        this.upSpeed = upSpeed;
        this.armGoalPosition = goal;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setArmGoalPosition(armGoal);
    }

    @Override
    public void execute() {

        armGoalPosition = arm.getArmGoalPosition();
        // if (!hasHit) {
        //     if (arm.getPosition() > armGoalPosition + DEADBAND)
        //         arm.setArmOpenLoop(-upSpeed);
        //     else if (arm.getPosition() < armGoalPosition - DEADBAND)
        //         arm.setArmOpenLoop(+downSpeed);
        //     else hasHit = true;
        // } else {
        //     if (arm.getPosition() - armGoalPosition > DEADBAND) {
        //         arm.setArmOpenLoop(-armGoal.getDeadbandPower());
        //     } else {
        //         arm.setArmOpenLoop(0);
        //     }

        //     if (Math.abs(arm.getPosition() - armGoalPosition) > DEADBAND * 15) {
        //         hasHit = false;
        //     }
        // }

        if (arm.getPosition() > armGoalPosition + UP_DEADBAND) {
            arm.setArmOpenLoop(-upSpeed);
        } else if (arm.getPosition() < armGoalPosition - DOWN_DEADBAND) {
            arm.setArmOpenLoop(+downSpeed);
        } else {
            arm.setArmOpenLoop(-calculateDeadbandSpeed(armGoalPosition));
        }
        
        // arm.setArmOpenLoop(-1.5 * (arm.getPosition() - armGoalPosition));
    }

    public double calculateDeadbandSpeed(double armGoalPosition) {
        if (arm.getPosition() > armGoalPosition) {
            return -(upSpeed - (upSpeed / (UP_DEADBAND)) * (arm.getPosition() - (armGoalPosition - UP_DEADBAND)));
        } 
        if (arm.getPosition() < armGoalPosition) {
            return -(downSpeed - (downSpeed / (DOWN_DEADBAND)) * (arm.getPosition() - (armGoalPosition - DOWN_DEADBAND)));
        }
        return 0;
    }
  
    @Override
    public void end(boolean interrupted) {
        arm.setArmOpenLoop(0);
    }

    @Override
    public boolean isFinished() {
        return false; // experiment with deadband
    }
}

