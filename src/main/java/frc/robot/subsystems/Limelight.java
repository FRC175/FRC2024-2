package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.Drive.Drive;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Arrays;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;



public final class Limelight implements Subsystem {
    /**
     * The single instance of {@link Drive} used to implement the "singleton" design pattern. A description of the
     * singleton design pattern can be found in the JavaDoc for {@link Drive::getInstance()}.
     */
    private static Limelight instance;

    public final NetworkTable table;

    public int pipe;
    
    private Limelight() {
        
        pipe = 0;
        table = NetworkTableInstance.getDefault().getTable("limelight");
        table.getEntry("getpipe").setNumber(pipe);
        
    }

    public String getJson() {
        String data = table.getEntry("json").getString("0");
        return data;
    }

    private double[] getJsonData(String data, String target, int num) {

        data = data.replace('"', '/');
        int startIndex = -1;
        int endIndex = -1;
        double[] out = new double[num];
        for (int i = 0; i < num; i++) {
            if(data.contains(target)) {
                startIndex = data.indexOf(target) + target.length();
                data = data.substring(startIndex);
                endIndex = data.indexOf(",");
                String stringData = data.substring(0, endIndex);
                double doubleData = (double)(Double.valueOf(stringData));
                out[i] = doubleData;
            } else {
                out[i] = -1.0;
            }
        }  
    
        return out;
    }


    public void switchPipe() {

        if (pipe == 0) pipe = 1;
        else pipe = 0;
        
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Distance to Speaker", DriverStation.getAlliance().get() == Alliance.Red ? getDistance(4) : getDistance(7));
        SmartDashboard.putNumber("Horizontal Angular Displacement", DriverStation.getAlliance().get() == Alliance.Red ? getHorizontalAngle(4) : getHorizontalAngle(7));
    }

    public double[] getVerticalAngle(String data) {
        double[] angles = new double [numTargetsDetected(data)];
       
            angles = getJsonData(data, "ty/:", numTargetsDetected(data));
      
        
        return angles;

    }

    private double[] getHorizontalAngle(String data) {
        double[] angles = getJsonData(data, "tx/:", numTargetsDetected(data));
        return angles;
    }

    public Double getHorizontalAngle(int id) {
        int[] ids = getTargetIds(getJson());
        for (int i = 0; i < ids.length; ++i) {
            if (ids[i] == id) {
                return getHorizontalAngle(getJson())[i];
            }
        }

        return 0.0;
    }

    public boolean targetDetected() {
        return table.getEntry("tv").getDouble(0) == 1; 
    }

    public int numTargetsDetected(String data) {
        int numDetected = 0;
        double[] ids = getJsonData(data, "ID/:", 2);
        for(double id : ids) {
            if ( id != 0) {
                numDetected++;
            }
        }


        return numDetected; 
    }
     
     public int[] getTargetIds(String data) {
        double[] ids = getJsonData(data, "ID/:", 2);
        int[] intIds = new int[ids.length];
        for (int i = 0; i < ids.length; ++i) {
            intIds[i] = (int)ids[i];
        }
        return intIds;
    }
    
    private double targetAngleDegrees(double ty){
        return ty + LimelightConstants.mountAngle;
         

    } 

    private double targetAngleRadians(double ty) {
       return targetAngleDegrees(ty)* (3.14159 / 180.0);
    }

    private double targetHeight(int tag) {
        return LimelightConstants.tagHeights[tag] - LimelightConstants.mountHeight;
    }

    public Double getDistance(int id) {
        int[] ids = getTargetIds(getJson());

        for (int i = 0; i < ids.length; ++i) {
            if (ids[i] == id) {
                return getDistance(new double[] {id})[0];
            }
        }

        return 0.0;
    }

    // public double getDistanceAssumingAngle(int id, double angle) {

    // }

    private double[] getDistance(double[] ids) {
        int numTargets = numTargetsDetected(getJson());
        double[] distances = new double[numTargets];
        int i = 0;
        for (double id : ids) {
            double d;
            try { d = targetHeight((int)(id))/(Math.tan(targetAngleRadians(getVerticalAngle(getJson())[i])));
                distances[i] = d;
            } catch (ArrayIndexOutOfBoundsException e) {}
            i++;
        }
        return distances;
    }

    public double getDistanceToCurrentAprilTag() {
        return targetHeight(4) / (Math.tan(Math.toRadians(table.getEntry("ty").getDouble(0.0))));
    }

    public double getHorizontalAngleToCurrentAprilTag() {
        return table.getEntry("tx").getDouble(0.0);
    }

    public double getDistanceOfPointBetween(double[] ids) {
        double distance = 0;
        if (numTargetsDetected(getJson()) == 2) {
            
            double height = (targetHeight(0) + targetHeight(1))/2;
            double a1 = targetAngleRadians(getVerticalAngle(getJson())[0]);
            double a2;
            a2 = targetAngleRadians(getVerticalAngle(getJson())[1]);
            double angle = ( a1 + a2)/2;
            distance = height/Math.tan(angle);
        } 
        return distance;
    }


    /**
     * <code>getInstance()</code> is a crucial part of the "singleton" design pattern. This pattern is used when there
     * should only be one instance of a class, which makes sense for Robot subsystems (after all, there is only one
     * drivetrain). The singleton pattern is implemented by making the constructor private, creating a static variable
     * of the type of the object called <code>instance</code>, and creating this method, <code>getInstance()</code>, to
     * return the instance when the class needs to be used.
     *
     * Usage:
     * <code>Drive drive = Drive.getInstance();</code>
     *
     * @return The single instance of {@link Drive}
     */
    public static Limelight getInstance() {
        if (instance == null) {
            instance = new Limelight();
        }

        return instance;
    }


}