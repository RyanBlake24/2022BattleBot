package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Drivetrain;

public class BackupAuton extends SequentialCommandGroup {
    public BackupAuton(Drivetrain drivetrain) {
        addCommands(
                new StartEndCommand(() -> drivetrain.tankDrive(-0.5, -0.5), () -> drivetrain.tankDrive(0, 0),
                        drivetrain)
                                .withTimeout(1.5));
    }
}
