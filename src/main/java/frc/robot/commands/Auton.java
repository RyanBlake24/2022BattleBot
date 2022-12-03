package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;

public class Auton extends SequentialCommandGroup {
    public Auton(Drivetrain drivetrain){
        addCommands(
            new PrintCommand("starting"),
            new TurnAngle(drivetrain, -45),
            new WaitCommand(1),
            new DriveStraight(drivetrain, 2),
            new PrintCommand("ending"));
    }
}
