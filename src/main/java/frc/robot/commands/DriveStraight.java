package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;

public class DriveStraight extends PIDCommand {
    private final Drivetrain driveTrain;

    public DriveStraight(Drivetrain drivetrain, double distance) {
        super(new PIDController(/* Need to do PID Tuning to find the values needed to get the controller */
                1.5, 0.0, 0.0), drivetrain::getPosition, distance, d -> drivetrain.tankDrive(d * 0.5, d * 0.5), drivetrain);
        this.driveTrain = drivetrain;
        addRequirements(drivetrain);

        getController().setTolerance(0.1);
    }

    @Override
    public void initialize() {
        super.initialize();
        this.driveTrain.end();
        this.driveTrain.resetGyro();
        this.driveTrain.resetEncoders();
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        this.driveTrain.hard_stop();
    }
}
