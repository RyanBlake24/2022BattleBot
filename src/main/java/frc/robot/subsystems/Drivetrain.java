package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;

//Drivetrain Class
public class Drivetrain extends SubsystemBase {
    public static Drivetrain instance;
    private CANSparkMax L_PRIMARY, R_PRIMARY;
    SlewRateLimiter slewRateLeft = new SlewRateLimiter(0.9);
    SlewRateLimiter slewRateRight = new SlewRateLimiter(0.9);
    private AHRS navx = new AHRS(SerialPort.Port.kMXP); // NOTE! You need to plug in the navx gyro thingy
    // into the kMXP, or 1 for short, for it to work. Doing usb or something with
    // mess things up

    public Drivetrain() {
        // Setting the motors and their inversion
        double ENCODER_DISTANCE_TO_METERS = Math.PI * Units.inchesToMeters(4) / 15.0;

        L_PRIMARY = new CANSparkMax(1, MotorType.kBrushless);
        R_PRIMARY = new CANSparkMax(2, MotorType.kBrushless);
        L_PRIMARY.getEncoder().setPositionConversionFactor(ENCODER_DISTANCE_TO_METERS);
        R_PRIMARY.getEncoder().setPositionConversionFactor(ENCODER_DISTANCE_TO_METERS);
        R_PRIMARY.setInverted(false);
        L_PRIMARY.setInverted(true);
        SmartDashboard.putData(navx);
    }

    /**
     * Set the left side of the motor's speed
     * 
     * @param speed
     */
    public void setLeftSpeed(double speed) {
        L_PRIMARY.set(slewRateLeft.calculate(speed * 0.6));
    }

    public void setRightSpeed(double speed) {
        R_PRIMARY.set(slewRateRight.calculate(speed * 0.6));
    }

    public double getLeftDistance() {
        return L_PRIMARY.getEncoder().getPosition();
    }

    public double getRightDistance() {
        return R_PRIMARY.getEncoder().getPosition();
    }

    /*
     * public void setSpeed(double leftSpeed, double rightSpeed){
     * setLeftSpeed(leftSpeed * 0.2);
     * setRightSpeed(rightSpeed * 0.2);
     * }
     */

    public void tankDrive(double leftSpeed, double rightSpeed) {
        setLeftSpeed(leftSpeed);
        setRightSpeed(rightSpeed);
    }

    // Arcade Drive Method
    /*
     * public void acradeDrive(double leftStick, double rightStick){
     * double leftArcade = leftStick - rightStick;
     * double rightArcade = leftStick + rightStick;
     * 
     * setSpeed(leftArcade * 0.4, rightArcade*0.4);
     * }
     */

    /**
     * Gets the current angle of the gyro.
     */
    public double getGyroAngle() {
        //System.out.println("NAVX Con? Gyro get angle.");
        //System.out.println(navx.isConnected());
        System.out.print("NAVX Angle: ");
        System.out.println(navx.getAngle());
        return navx.getAngle();
    }

    public void end() {
        tankDrive(0, 0);
    }

    public void hard_stop() {
        L_PRIMARY.set(0);
        R_PRIMARY.set(0);
    }

    public void resetGyro() {
        navx.reset();
    }

    public double getPosition() {
        return (double) (L_PRIMARY.getEncoder().getPosition() + R_PRIMARY.getEncoder().getPosition())
                / 2.0;
    }

    public void resetEncoders() {
        L_PRIMARY.getEncoder().setPosition(0);
        R_PRIMARY.getEncoder().setPosition(0);
    }

}
