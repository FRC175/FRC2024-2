package frc.robot.utils;

public class EncoderValue {
    private int rotations;
    private double value;

    public EncoderValue(int rotations, double value) {
        this.rotations = rotations;
        this.value = value;
    }

    public int getRotations() {
        return rotations;
    }

    public double getValue() {
        return value;
    }

    public void addRotations(int rotationsToAdd) {
        rotations += rotationsToAdd;
    }

    public void setValue(double val) {
        value = val;
    }

    public double getDouble() {
        return value + rotations;
    }


}
