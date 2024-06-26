package frc.robot.subsystems.Drive;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.Vector;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public final class Drive implements Subsystem {

    // These variables are final because they only need to be instantiated once (after all, you don't need to create a
    // new left master TalonSRX).
    SwerveModule frontRight, frontLeft, backRight, backLeft;
    double lastValidAngle;

    Pigeon2 pigeon;

    Translation2d frontRightLocation, frontLeftLocation, backRightLocation, backLeftLocation;
    SwerveDriveKinematics kinematics;
    SwerveDriveOdometry odometry;
    Pose2d pose;
    
    /**
     * The single instance of {@link Drive} used to implement the "singleton" design pattern. A description of the
     * singleton design pattern can be found in the JavaDoc for {@link Drive::getInstance()}.
     */
    private static Drive instance;
    
    private Drive() {
        // leftMaster = new CANSparkMax(DriveConstants.LEFT_MASTER_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);
        frontRight = new SwerveModule(DriveConstants.frontRightDrive, DriveConstants.frontRightRot, DriveConstants.frontRightEncoder, DriveConstants.frontRightTurnAngle, DriveConstants.frontRightBaseAngle, true);
        frontLeft = new SwerveModule(DriveConstants.frontLeftDrive, DriveConstants.frontLeftRot, DriveConstants.frontLeftEncoder, DriveConstants.frontLeftTurnAngle, DriveConstants.frontLeftBaseAngle, false);
        backRight = new SwerveModule(DriveConstants.backRightDrive, DriveConstants.backRightRot, DriveConstants.backRightEncoder, DriveConstants.backRightTurnAngle, DriveConstants.backRightBaseAngle, false);
        backLeft = new SwerveModule(DriveConstants.backLeftDrive, DriveConstants.backLeftRot, DriveConstants.backLeftEncoder, DriveConstants.backLeftTurnAngle, DriveConstants.backLeftBaseAngle, true);
        configureSparks();

        pigeon = new Pigeon2(DriveConstants.PIDGEON);

        // SwerveModule test = new SwerveModule

        pose = new Pose2d(0, 0, new Rotation2d());

        final double HALF_WHEEL_BASE_WIDTH = 0.299; // meters


        frontRightLocation = new Translation2d(HALF_WHEEL_BASE_WIDTH, -HALF_WHEEL_BASE_WIDTH);
        frontLeftLocation = new Translation2d(HALF_WHEEL_BASE_WIDTH, HALF_WHEEL_BASE_WIDTH);
        backRightLocation = new Translation2d(-HALF_WHEEL_BASE_WIDTH, -HALF_WHEEL_BASE_WIDTH);
        backLeftLocation = new Translation2d(-HALF_WHEEL_BASE_WIDTH, HALF_WHEEL_BASE_WIDTH);

        kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
        odometry = new SwerveDriveOdometry(
            kinematics, new Rotation2d(Math.toRadians(getOdometryAngle())),
            new SwerveModulePosition[] {
              new SwerveModulePosition(0, new Rotation2d()),
              new SwerveModulePosition(0, new Rotation2d()),
              new SwerveModulePosition(0, new Rotation2d()),
              new SwerveModulePosition(0, new Rotation2d())
            }, pose);

        lastValidAngle = 0;

        resetDistance();
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
    public static Drive getInstance() {
        if (instance == null) {
            instance = new Drive();
        }

        return instance;
    }

    /**
     * Helper method that configures the Spark Max motor controllers.
     */
    private void configureSparks() {

        
    }

    public Rotation2d getGyro() {
        return new Rotation2d(Math.toRadians(getOdometryAngle()));
    }

    /**
     * Sets the drive motors to a certain percent output (demand) using open loop control (no sensors in feedback loop).
     *
     * @param leftDemand The percent output for the left drive motors
     * @param rightDemand The percent output for the right drive motors
     */
    public void setOpenLoop(double drive, double turn) {
        frontRight.driveOpenLoop(drive);
        frontLeft.driveOpenLoop(drive);
        backRight.driveOpenLoop(drive);
        backLeft.driveOpenLoop(drive);

        frontRight.turnOpenLoop(turn);
        frontLeft.turnOpenLoop(turn);
        backRight.turnOpenLoop(turn);
        backLeft.turnOpenLoop(turn);
    }

    @Override
    public void periodic() {
        // Get the rotation of the robot from the gyro.
        var gyroAngle = pigeon.getRotation2d();

        // System.out.println(gyroAngle.getDegrees());

        // Update the pose
        pose = odometry.update(new Rotation2d(Math.toRadians(getOdometryAngle())),
        new SwerveModulePosition[] {
            new SwerveModulePosition(frontLeft.getDriveDistance(), new Rotation2d(Math.toRadians(frontLeft.getOdometryAngle()))), new SwerveModulePosition(frontRight.getDriveDistance(), new Rotation2d(Math.toRadians(frontRight.getOdometryAngle()))),
            new SwerveModulePosition(backLeft.getDriveDistance(), new Rotation2d(Math.toRadians(backLeft.getOdometryAngle()))), new SwerveModulePosition(backRight.getDriveDistance(), new Rotation2d(Math.toRadians(backRight.getOdometryAngle())))
        });

        // System.out.println("FL: " + frontLeft.getDriveDistance() + "\t\t" + frontLeft.isReversed());
        // System.out.println("FR: " + frontRight.getDriveDistance() + "\t\t" + frontRight.isReversed());
        // System.out.println("BL: " + backLeft.getDriveDistance() + "\t\t" + backLeft.isReversed());
        // System.out.println("BR: " + backRight.getDriveDistance() + "\t\t" + backRight.isReversed());

        // SmartDashboard.putNumber("X Position: ", getPose().getX());
        // SmartDashboard.putNumber("Y Position: ", getPose().getY());
        // SmartDashboard.putNumber("Front Left Raw", frontLeft.getOdometryAngle());
        SmartDashboard.putNumber("Toddler Mode Level: ", Constants.TODDLER_MODE);


        // SmartDashboard.putNumber("Distance: ", frontLeft.getDriveDistance());

        postYaw();

    }

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(new Rotation2d(Math.toRadians(getOdometryAngle())),
            new SwerveModulePosition[] {
            new SwerveModulePosition(frontLeft.getDriveDistance(), new Rotation2d(Math.toRadians(frontLeft.getOdometryAngle()))), new SwerveModulePosition(frontRight.getDriveDistance(), new Rotation2d(Math.toRadians(frontRight.getOdometryAngle()))),
            new SwerveModulePosition(backLeft.getDriveDistance(), new Rotation2d(Math.toRadians(backLeft.getOdometryAngle()))), new SwerveModulePosition(backRight.getDriveDistance(), new Rotation2d(Math.toRadians(backRight.getOdometryAngle())))
        }, pose);
    }

    // public ChassisSpeeds[] getRobotRelativeSpeeds() {
    //     return new ChassisSpeeds[] {new ChassisSpeeds()}
    // }

    public void swerve(double joyX, double joyY, double twist, double gyroAngle) {
        Vector transversal = new Vector(joyX, joyY * -1);
        // SmartDashboard.putNumber("Joystick Angle", transversal.getAngle());

        if (transversal.getMagnitude() < 0.001) {
            transversal.setAngle(lastValidAngle);
        } else {
            lastValidAngle = transversal.getAngle();
        }

        transversal.rotate(- 90 - gyroAngle);

        frontRight.calculateRawOutputs(transversal, frontRight.findControlledRotationVector(twist));
        frontLeft.calculateRawOutputs(transversal, frontLeft.findControlledRotationVector(twist));
        backRight.calculateRawOutputs(transversal, backRight.findControlledRotationVector(twist));
        backLeft.calculateRawOutputs(transversal, backLeft.findControlledRotationVector(twist));

        frontRight.setOutputs();
        frontLeft.setOutputs();
        backRight.setOutputs();
        backLeft.setOutputs();
    }

    public void lockSwerve(double joyX, double joyY, double lockAngle, double gyroAngle) {
        Vector transversal = new Vector(joyX, joyY * -1);
        // SmartDashboard.putNumber("Joystick Angle", transversal.getAngle());

        if (transversal.getMagnitude() < 0.001) {
            transversal.setAngle(lastValidAngle);
        } else {
            lastValidAngle = transversal.getAngle();
        }

        transversal.rotate(- 90 - gyroAngle);

        frontRight.calculateRawOutputs(transversal, frontRight.findLockedRotationVector(lockAngle, gyroAngle));
        frontLeft.calculateRawOutputs(transversal, frontLeft.findLockedRotationVector(lockAngle, gyroAngle));
        backRight.calculateRawOutputs(transversal, backRight.findLockedRotationVector(lockAngle, gyroAngle));
        backLeft.calculateRawOutputs(transversal, backLeft.findLockedRotationVector(lockAngle, gyroAngle));

        frontRight.setOutputs();
        frontLeft.setOutputs();
        backRight.setOutputs();
        backLeft.setOutputs();
    }

    public void lock() {
        frontRight.lock();
        frontLeft.lock();
        backRight.lock();
        backLeft.lock();

        frontRight.setOutputs();
        frontLeft.setOutputs();
        backRight.setOutputs();
        backLeft.setOutputs();
    }

    public void turning(double fr, double fl, double br, double bl) {
        frontRight.turnOpenLoop(fr);
        frontLeft.turnOpenLoop(fl);
        backRight.turnOpenLoop(br);
        backLeft.turnOpenLoop(bl);
    }

    public double getFRPosition() {
        return frontRight.getDriveEncoder();
    }
    public double getFLPosition() {
        return frontLeft.getDriveDistance();
    }
    public double getBRPosition() {
        return backRight.getDriveEncoder();
    }
    public double getBLPosition() {
        return backLeft.getDriveEncoder();
    }

    public double getDriveDistance() {
        return frontLeft.getDriveDistance();
    }

    public void resetDriveDistance() {
        frontLeft.resetDriveEncoder();
    }

    public double getYaw() {
        return ((pigeon.getYaw().getValue() % 360) + 360) % 360;
    }

    public double getOdometryAngle() {
        if (getYaw() > 180) {
            return -(360-getYaw());
        } else {
            return getYaw();
        }
    }

    public void resetGyro(double val) {
        pigeon.setYaw(val);
    }

    public void postYaw() {
        SmartDashboard.putNumber("Yee-Yaw", getYaw());
    }

    public void resetDistance() {
        frontLeft.resetDriveEncoder();
    }

    public Translation2d getPose() {
        return pose.getTranslation();
    }

   
}
