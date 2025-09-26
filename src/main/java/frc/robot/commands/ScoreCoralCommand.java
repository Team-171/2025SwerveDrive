
package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ScorePositionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ScoreCoralCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final ScorePositionSubsystem m_scoreSubsystem;
    private final ArmSubsystem m_ArmSubsystem;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ScoreCoralCommand(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, ScorePositionSubsystem scorePositionSubsystem) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_scoreSubsystem = scorePositionSubsystem;
        m_ArmSubsystem = armSubsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(elevatorSubsystem, scorePositionSubsystem, armSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_elevatorSubsystem.setHoldPosition(m_scoreSubsystem.getLevelElevatorHoldPosition());
        //m_ArmSubsystem.setHoldPosition(m_scoreSubsystem.getLevelArmPosition());
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
