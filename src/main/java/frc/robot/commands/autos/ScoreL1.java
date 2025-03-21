// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralConstants;
import frc.robot.subsystems.CoralSubsystem;

public class ScoreL1 extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final CoralSubsystem coralSubsystem;
    // Distance to drive forward
    // If the robot reached the distance

    private double time;

    /**
     * Creates the drive forward auto
     * Drives forward a set distance
     * 
     * @param distance double The distance to drive
     * @param slow     boolean If the robot should drive slower
     */
    public ScoreL1(CoralSubsystem coralSubsystem) {
        this.coralSubsystem = coralSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(coralSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        time = Timer.getFPGATimestamp();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Drives forward and returns if it reached the destination
        coralSubsystem.outputCoral(CoralConstants.kCoralSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // When the robot is at the destination, set the speed to 0
        coralSubsystem.stopMotion();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (Timer.getFPGATimestamp() - time >= 1)
            return true;
        return false;
    }
}
