package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;

public class TurnAngle extends PIDCommand {
    private final Drivetrain driveTrain;

    public TurnAngle(Drivetrain drivetrain, double setPoint) {
        super(new PIDController(/* Need to do PID Tuning to find the values needed to get the controller */
                0.15, 0.0, 0), drivetrain::getGyroAngle, setPoint, d -> drivetrain.tankDrive(d, -d), drivetrain);
        this.driveTrain = drivetrain;
        addRequirements(drivetrain);
        getController().setTolerance(2);
    }

    @Override
    public void initialize() {
        this.driveTrain.stop();
        this.driveTrain.resetGyro();
        this.driveTrain.resetEncoders();
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}
