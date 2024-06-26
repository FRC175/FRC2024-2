package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Limelight;

public class ArmViaLimelight extends Command {

    Arm arm;
    Limelight limelight;
    int regression;

    public ArmViaLimelight(Limelight limelight, Arm arm, int regression) {
        this.arm = arm;
        this.limelight = limelight;
        this.regression = regression;
        
    }

    public double calcAngleDisplacement() {
        int id = DriverStation.getAlliance().get() == Alliance.Red ? 4 : 7;
        double angle = limelight.getHorizontalAngle(id);
        double distance = limelight.getDistance(id);
        double shift = 4.5;
        double new_distance = Math.sqrt(Math.pow(distance*Math.sin(Math.toRadians(angle)) + shift, 2) + Math.pow(distance*Math.cos(Math.toRadians(angle)),2));
        double new_angle = Math.toDegrees(Math.atan2(distance*Math.sin(Math.toRadians(angle)) + shift, distance*Math.cos(Math.toRadians(angle))));
        double output = 90 - Math.toDegrees(Math.atan2(new_distance * Math.cos(Math.toRadians(new_angle)), new_distance * Math.sin(Math.toRadians(new_angle)) - 8.5));
        return output;
    }

    @Override
    public void execute() {
        double output = 0;
        double d = DriverStation.getAlliance().get() == Alliance.Red ? limelight.getDistance(4) : limelight.getDistance(7);
        double x = d/Math.cos(Math.toRadians(calcAngleDisplacement()));
        // double x = limelight.getDistanceToCurrentAprilTag();
        //output = 0.894561 - 0.00275256*x + 0.0000153065*x*x;
        // output = 0.894561 - 0.00275256*x + 0.0000153065*x*x;
        if (regression == 0) {
            output = 0.0000121765*x*x - 0.00237629*x + 0.883725;
            // output = 0.836466 - 0.000823962*x;
            // output = 0.829904 - 0.000530378*x - 2.54944*Math.pow(10,-6)*x*x;
            // output = 0.788769 + 0.000795115*x - 0.0000121232*x*x;
            // output = 0.894561 - 0.00275256*x + 0.0000153065*x*x;
        } else if (regression == 1) {
            output = 0.0000121765*x*x - 0.00237629*x + 0.883725 - 0.005;
            // output = -0.0000583692*x*x + 0.00629835*x + 0.622693;
        } else if (regression == 2) {
            output = 0.0000138131*x*x - 0.00253024*x + 0.889074;
        }
        

        arm.setArmGoalPosition(output);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
