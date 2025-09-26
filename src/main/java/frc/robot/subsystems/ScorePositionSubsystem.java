package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helperObjects.DriveSideEnum;
import frc.helperObjects.StateEnum;

public class ScorePositionSubsystem extends SubsystemBase {

    static StateEnum levelPosition = StateEnum.CORAL_LEVEL_1;
    DriveSideEnum side = DriveSideEnum.LEFT;

    public ScorePositionSubsystem() {
    }

    // Set the score position to a coral level
    public void setScoreLevel(StateEnum level) {
        levelPosition = level;
    }

    public void setScoreLeftOrRight(DriveSideEnum driveSideEnum) {
        side = driveSideEnum;
    }

    public DriveSideEnum getDriveSide() {
        return side;
    }

    public double getLevelElevatorHoldPosition() {
        return levelPosition.elevatorValue;
    }

    public double getLevelArmPosition() {
        return levelPosition.armValue;
    }

    public StateEnum getLevelState() {
        SmartDashboard.putString("Returned State", levelPosition.toString());
        return levelPosition;
    }

    public int getLevel() {
        return levelPosition.ordinal() + 1; // Translate from zero-indexed to one-indexed
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Selected Level", levelPosition.toString());
        SmartDashboard.putString("Selected Side", side.toString());
    }
}
