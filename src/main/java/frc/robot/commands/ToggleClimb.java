// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.LiftConstants;
// import frc.robot.subsystems.Lift;

// public class ToggleClimb extends Command {
    
//     private Lift lift;
//     private double speed;
//     private double LEFT_DEADBAND = 0.05;
//     private double RIGHT_DEADBAND = 0.05;


//     public ToggleClimb(Lift lift) {
//         this.lift = lift;
        
//         addRequirements(lift);
//     }

//     @Override
//     public void initialize() {
//         if (lift.getLeftGoalPosition() == LiftConstants.STARTING_POSITION_LEFT) {
//             lift.setLeftGoalPosition(LiftConstants.FINAL_POSITION_LEFT);
//         } else {
//             lift.setLeftGoalPosition(LiftConstants.STARTING_POSITION_LEFT);
//         }
//         if (lift.getRightGoalPosition() == LiftConstants.STARTING_POSITION_RIGHT) {
//             lift.setLeftGoalPosition(LiftConstants.FINAL_POSITION_LEFT);
//         } else {
//             lift.setLeftGoalPosition(LiftConstants.STARTING_POSITION_LEFT);
//         }
//     }

//     @Override
//     public void execute() {

        

//     }
  
//     @Override
//     public void end(boolean interrupted) {
//         lift.setLeftOpenLoop(0);
//         lift.setRightOpenLoop(0);
//     }

//     @Override
//     public boolean isFinished() {
//         return false; // experiment with deadband
//     }
// }

