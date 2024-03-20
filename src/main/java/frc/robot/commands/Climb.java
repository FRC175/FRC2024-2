package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lift;
import frc.robot.utils.EncoderValue;

public class Climb extends Command {
    
    private Lift lift;
    private double speed;
    private double LEFT_DEADBAND = 0.1;
    private double RIGHT_DEADBAND = 0.1;


    public Climb(Lift lift, double speed) {
        this.lift = lift;
        this.speed = speed;
        
        addRequirements(lift);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        double leftGoalPosition = lift.getLeftGoalPosition();
        double rightGoalPosition = lift.getRightGoalPosition();

        
        if (lift.getLeftValue() > leftGoalPosition + LEFT_DEADBAND) {

            lift.setLeftOpenLoop(+speed);
        } else if (lift.getLeftValue() < leftGoalPosition - LEFT_DEADBAND) {
            lift.setLeftOpenLoop(-speed);
        } else {
            lift.setLeftOpenLoop(calculateLeftDeadbandSpeed(leftGoalPosition));
        }

        if (lift.getRightValue() > rightGoalPosition + RIGHT_DEADBAND) {
            lift.setRightOpenLoop(+speed);
        } else if (lift.getRightValue() < rightGoalPosition - RIGHT_DEADBAND) {
            lift.setRightOpenLoop(-speed);
        } else {
            lift.setRightOpenLoop(calculateRightDeadbandSpeed(rightGoalPosition));
        }

    }

    public double calculateLeftDeadbandSpeed(double liftGoalPosition) {
        // if (lift.getLeftValue() > liftGoalPosition) {
        //     return -(speed - (speed / (LEFT_DEADBAND)) * (lift.getLeftValue() - (liftGoalPosition + LEFT_DEADBAND)));
        // } 
        // if (lift.getLeftValue() < liftGoalPosition) {
        //     return speed - (speed / (LEFT_DEADBAND)) * (lift.getLeftValue() - (liftGoalPosition - LEFT_DEADBAND));
        // }

        return (speed / LEFT_DEADBAND) * (lift.getLeftValue() - liftGoalPosition);
    }

    public double calculateRightDeadbandSpeed(double liftGoalPosition) {
        // if (lift.getRightValue() > liftGoalPosition) {
        //     return -(speed - (speed / (RIGHT_DEADBAND)) * (lift.getRightValue() - (liftGoalPosition + RIGHT_DEADBAND)));
        // } 
        // if (lift.getRightValue() < liftGoalPosition) {
        //     return speed - (speed / (RIGHT_DEADBAND)) * (lift.getRightValue() - (liftGoalPosition - RIGHT_DEADBAND));
        // }
        // return 0;
        return (speed / RIGHT_DEADBAND) * (lift.getRightValue() - liftGoalPosition);
    }

    // public double calculateDeadbandSpeed(double armGoalPosition) {
       

        
    // }
  
    @Override
    public void end(boolean interrupted) {
        lift.setLeftOpenLoop(0);
        lift.setRightOpenLoop(0);
    }

    @Override
    public boolean isFinished() {
        return false; // experiment with deadband
    }
}

