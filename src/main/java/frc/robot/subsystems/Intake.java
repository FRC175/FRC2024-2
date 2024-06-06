package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.IntakeConstants;

public class Intake implements Subsystem {
    
    private final CANSparkMax intakeMotor; 
    private final RelativeEncoder intakeMotorEncoder; 
    private final DigitalInput reflectometer, electricBoogaloo; 
    private static Intake instance; 

    private Intake() {
        intakeMotor = new CANSparkMax(IntakeConstants.INTAKE, CANSparkMaxLowLevel.MotorType.kBrushless);
        intakeMotor.setInverted(true);
        // intakeMotor.setSmartCurrentLimit(80);
        intakeMotor.restoreFactoryDefaults();
        intakeMotorEncoder = intakeMotor.getEncoder(); 
        reflectometer = new DigitalInput(0);
        electricBoogaloo = new DigitalInput(2);
    }

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }

        return instance; 
    }
    //hello

    @Override
    public void periodic() {
        // System.out.println(isNotePresent());
        SmartDashboard.putBoolean("Pickup Sensor", isNotePresent());
        SmartDashboard.putBoolean("Electric Boogaloo", isNoteHeld());
        // System.out.println(intakeMotor.getOutputCurrent());
    }

    public void setOpenLoop(double demand) {
        intakeMotor.set(demand);
    }

    public double getMotorRPM() {
        return intakeMotorEncoder.getVelocity(); 
    }

    public boolean isNotePresent() {
        return reflectometer.get();
    }

    public boolean isNoteHeld() {
        return electricBoogaloo.get();
    }
}
