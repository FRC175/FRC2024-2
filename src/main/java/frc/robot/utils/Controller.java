package frc.robot.utils;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Controller {
    private Joystick joy;
    private XboxController controller;
    boolean isJoystick = true;

    public Controller (Joystick joy) {
        this.joy = joy;
        isJoystick = true;
        controller = null;
    }

    public Controller (XboxController controller) {
        this.controller = controller;
        this.joy = null;
        isJoystick = false;
    }

    public double getLeftX() {
        if (isJoystick) {
            return joy.getX();
        } else {
            return controller.getLeftX();
        }
    }

    public boolean get4() {
        if (isJoystick) return new JoystickButton(joy, 4).getAsBoolean();
        else return controller.getYButton();
    }

    public boolean get6() {
        if (isJoystick) return new JoystickButton(joy, 6).getAsBoolean();
        else return false;
    }

    public boolean get2() {
        if (isJoystick) return new JoystickButton(joy, 2).getAsBoolean();
        else return controller.getXButton();
    }

    public boolean get3() {
        if (isJoystick) return new JoystickButton(joy, 3).getAsBoolean();
        else return false;
    }

    public double getLeftY() {
        if (isJoystick) {
            return joy.getY();
        } else {
            return controller.getLeftY();
        }
    }

    public double getTwist() {
        if (isJoystick) {
            return joy.getTwist();
        } else {
            return controller.getRightX();
        }
    }

    public boolean getB12() {
        if (isJoystick) {
            return new JoystickButton(joy, 12).getAsBoolean();
        } else {
            return controller.getBButton();
        }
    }

    public boolean getA11() {
        if (isJoystick) {
            return new JoystickButton(joy, 11).getAsBoolean();
        } else {
            return controller.getAButton();
        }
    }

    public double getPOV() {
        if (isJoystick) {
            return joy.getPOV();
        } else {
            return controller.getPOV();
        }
    }

    public boolean getTrigger() {
        if (isJoystick) {
            return joy.getTrigger();
        } else {
            return controller.getRightTriggerAxis() > 0.5;
        }
    }
}
