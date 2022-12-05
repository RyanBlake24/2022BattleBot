package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;

//Drivetrain Class
public class Drivetrain extends SubsystemBase {
    public static Drivetrain instance;
    private CANSparkMax L_PRIMARY, R_PRIMARY;
    DifferentialDrive drive;
    private AHRS navx = new AHRS(SerialPort.Port.kMXP); // NOTE! You need to plug in the navx gyro thingy
    // into the kMXP, or 1 for short, for it to work. Doing usb or something with
    // mess things up

    /**
     * Initalizes the Drivetrain and its components.
     */
    public Drivetrain() {
        // Setting the motors and their inversion
        double ENCODER_DISTANCE_TO_METERS = Math.PI * Units.inchesToMeters(4) / 15.0;

        L_PRIMARY = new CANSparkMax(1, MotorType.kBrushless);
        R_PRIMARY = new CANSparkMax(2, MotorType.kBrushless);
        drive = new DifferentialDrive(L_PRIMARY, R_PRIMARY);
        L_PRIMARY.getEncoder().setPositionConversionFactor(ENCODER_DISTANCE_TO_METERS);
        R_PRIMARY.getEncoder().setPositionConversionFactor(ENCODER_DISTANCE_TO_METERS);
        R_PRIMARY.setInverted(false);
        L_PRIMARY.setInverted(true);
        SmartDashboard.putData(navx);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Distance", getPosition());
        SmartDashboard.putNumber("Angle", navx.getAngle());
    }

    /**
     * Gets the position (or number of rotations) of the left motor's encoder.
     * 
     * @return The position of the left motor's encoder
     */
    public double getLeftDistance() {
        return L_PRIMARY.getEncoder().getPosition();
    }

    /**
     * Gets the position (or number of rotations) of the right motor's encoder.
     * 
     * @return The position of the right motor's encoder
     */
    public double getRightDistance() {
        return R_PRIMARY.getEncoder().getPosition();
    }

    /**
     * Runs the drivetrain in Tank Drive Mode. Aka left joystick is left side, right
     * joystick is right
     * side.
     * 
     * @param leftSpeed  The speed from -1 to 1 to set the left motor. Typically
     *                   taken from the left
     *                   joystick's y-axis.
     * @param rightSpeed The speed from -1 to 1 to set the right motor. Typically
     *                   taken from the right
     *                   joystick's y-axis.
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        drive.tankDrive(leftSpeed, rightSpeed);
    }

    /**
     * Runs the drivetrain in Arcade Drive Mode. Aka one joystick controls it's
     * movement up and down,
     * while the other controls it's movement left and right.
     * 
     * @param xSpeed The speed from -1 to 1 to set both motors to. Typically taken
     *               from the left
     *               joystick's y-axis.
     * @param rot    The rotation from -1 to 1 to rotate the robot to. Typically
     *               taken from the right
     *               joystick's x-axis.
     */
    public void arcadeDrive(double xSpeed, double rot) {
        drive.arcadeDrive(xSpeed, rot);
    }

    /**
     * Gets the current angle of the gyro.
     */
    public double getGyroAngle() {
        return navx.getAngle();
    }

    /**
     * Sets all motor's speed to zero
     */
    public void stop() {
        tankDrive(0, 0);
    }

    /**
     * Resets the gyro's yaw axis to 0 for what direction it is currently facing.
     */
    public void resetGyro() {
        navx.reset();
    }

    /**
     * Gets the average position of the left and right motor's encoders
     * 
     * @return The average position of the left and right encoders
     */
    public double getPosition() {
        return (double) (L_PRIMARY.getEncoder().getPosition() + R_PRIMARY.getEncoder().getPosition())
                / 2.0;
    }

    /**
     * Resets the encoder's rotations back to zero.
     */
    public void resetEncoders() {
        L_PRIMARY.getEncoder().setPosition(0);
        R_PRIMARY.getEncoder().setPosition(0);
    }

}
