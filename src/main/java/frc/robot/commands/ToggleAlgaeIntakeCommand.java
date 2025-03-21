// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.helperObjects.AlgaeScoreLevelEnum;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorPositionConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ToggleAlgaeIntakeCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final ArmSubsystem m_armSubsystem;
    private static AlgaeScoreLevelEnum scoreLevel;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ToggleAlgaeIntakeCommand(ElevatorSubsystem subsystem, ArmSubsystem armSubsystem) {
        m_elevatorSubsystem = subsystem;
        m_armSubsystem = armSubsystem;
        scoreLevel = AlgaeScoreLevelEnum.HIGH;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_armSubsystem.setHoldPosition(ArmConstants.kIntakeAlgaePosition);
        // toggle level
        switch (scoreLevel) {
            case LOW:
                scoreLevel = AlgaeScoreLevelEnum.HIGH;
                break;
            case HIGH:
                scoreLevel = AlgaeScoreLevelEnum.LOW;
                break;
        }

        // set elevator hold position
        switch (scoreLevel) {
            case LOW:
                m_elevatorSubsystem.setHoldPosition(ElevatorPositionConstants.kIntakeAlgae23Position);
                break;
            case HIGH:
                m_elevatorSubsystem.setHoldPosition(ElevatorPositionConstants.kIntakeAlgae34Position);
                break;
            default:
                m_elevatorSubsystem.setHoldPosition(ElevatorPositionConstants.kIntakeAlgae23Position);
                break;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
