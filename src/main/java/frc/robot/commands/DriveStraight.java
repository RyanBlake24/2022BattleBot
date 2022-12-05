package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;

public class DriveStraight extends PIDCommand {
    private final Drivetrain drivetrain;

    public DriveStraight(Drivetrain drivetrain, double distance, double speedFactor) {
        super(new PIDController(/* Need to do PID Tuning to find the values needed to get the controller */
                1.5, 0.0, 0.0), drivetrain::getPosition, distance, d -> drivetrain.tankDrive(d * speedFactor, d * speedFactor), drivetrain);
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        getController().setTolerance(0.1);
    }

    @Override
    public void initialize() {
        super.initialize();
        drivetrain.stop();
        drivetrain.resetGyro();
        drivetrain.resetEncoders();
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}
