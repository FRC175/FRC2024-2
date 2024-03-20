package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.Constants.LEDConstants;

public final class LED implements Subsystem {

    private final Spark blinkin;

    private static LED instance;

    private LEDColor currentColor;

    private LED() {
        blinkin = new Spark(LEDConstants.BLINKIN_PORT);
        currentColor = LEDColor.YELLOW;
    }

    public enum LEDColor {
        HOT_PINK(0.57),
        ORANGE(0.65),
        LIME(0.73),
        BLACK(0.99),
        BLUE(0.85),
        PARTYMODE(-0.53),
        RED(0.61),
        YELLOW(0.69);

        private double val;

        private LEDColor(double val) {
            this.val = val;
        }

        public double getVal() {
            return val;
        }
    }

    public static LED getInstance() {
        if (instance == null) {
            instance = new LED();
        }

        return instance;
    }

    public LEDColor getColor() {
        return currentColor;
    }

    public void setColor(LEDColor color) {
        currentColor = color;
    }

    public void setBlinkin(double value) {
        blinkin.set(value);
    }



    @Override
    public void periodic() {
        setBlinkin(currentColor.getVal());
    }



}