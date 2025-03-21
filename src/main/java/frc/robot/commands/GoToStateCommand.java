
package frc.robot.commands;

import frc.helperObjects.StateEnum;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class GoToStateCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final ArmSubsystem m_armSubsystem;
    private final StateEnum state;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public GoToStateCommand(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, StateEnum state) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_armSubsystem = armSubsystem;
        this.state = state;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_elevatorSubsystem, m_armSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_elevatorSubsystem.setHoldPosition(state.elevatorValue);
        m_armSubsystem.setHoldPosition(state.armValue);
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
