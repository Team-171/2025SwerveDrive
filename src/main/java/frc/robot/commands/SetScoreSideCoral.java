
package frc.robot.commands;

import frc.helperObjects.DriveSideEnum;
import frc.robot.subsystems.ScorePositionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class SetScoreSideCoral extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final ScorePositionSubsystem m_subsystem;
    private final DriveSideEnum side;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public SetScoreSideCoral(ScorePositionSubsystem subsystem, DriveSideEnum side) {
        m_subsystem = subsystem;
        this.side = side;
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
        m_subsystem.setScoreLeftOrRight(side);
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
