package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.Constants.LiftConstants;
import frc.robot.utils.EncoderValue;;

public class Lift implements Subsystem  {
    
    private final CANSparkMax leftLift, rightLift; 

    private double leftGoalPosition;
    private double rightGoalPosition;

    private final DutyCycleEncoder leftEncoder, rightEncoder; 

    private final EncoderValue leftValue, rightValue;
    private double lastLeft, lastRight;

    private static Lift instance; 

    private Lift() {
        leftLift = new CANSparkMax(LiftConstants.LEFT, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightLift = new CANSparkMax(LiftConstants.RIGHT, CANSparkMaxLowLevel.MotorType.kBrushless);

        leftEncoder = new DutyCycleEncoder(3);
        rightEncoder = new DutyCycleEncoder(4);

        leftGoalPosition = LiftConstants.STARTING_POSITION_LEFT;
        rightGoalPosition = LiftConstants.STARTING_POSITION_RIGHT;

        leftValue = new EncoderValue(0, getLeftPosition());
        rightValue = new EncoderValue(0, getRightPosition());
        lastLeft = getLeftPosition();
        lastRight = getRightPosition();
    }

    public static Lift getInstance() {
        if (instance == null) {
            instance = new Lift();
        }

        return instance; 
    } 

    public double getRightValue() {
        return rightValue.getDouble();
    }

    public double getLeftValue() {
        return leftValue.getDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb Left", getLeftPosition());
        SmartDashboard.putNumber("Climb Right", getRightPosition());

        if (getLeftPosition() - lastLeft > 0.9) {
            leftValue.addRotations(1);
        } else if (getLeftPosition() - lastLeft < -0.9) {
            leftValue.addRotations(-1);
        }

        if (getRightPosition() - lastRight > 0.9) {
            rightValue.addRotations(-1);
        } else if (getRightPosition() - lastRight < -0.9) {
            rightValue.addRotations(1);
        }

        leftValue.setValue(1-getLeftPosition());
        rightValue.setValue(getRightPosition());

        SmartDashboard.putNumber("Left Value", leftValue.getDouble());
        SmartDashboard.putNumber("Right Value", rightValue.getDouble());

        SmartDashboard.putNumber("Left Goal", leftGoalPosition);
        SmartDashboard.putNumber("Right Goal", rightGoalPosition);

        lastLeft = getLeftPosition();
        lastRight = getRightPosition();
    }

    public void setLeftOpenLoop(double demand) {
        leftLift.set(demand);
    }

    public void setRightOpenLoop(double demand) {
        rightLift.set(-demand);
    }

    public void setRightGoalPosition(double position) {
        rightGoalPosition = position;
    }

    public void setLeftGoalPosition(double position) {
        leftGoalPosition = position;
    }

    public double getRightGoalPosition() {
        return rightGoalPosition;
    }

    public double getLeftGoalPosition() {
        return leftGoalPosition;
    }

    public double getRightPosition() {
        return rightEncoder.getAbsolutePosition();
    }

    public double getLeftPosition() {
        return leftEncoder.getAbsolutePosition();
    }

}
