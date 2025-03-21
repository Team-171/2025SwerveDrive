package frc.helperObjects;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorPositionConstants;

public enum StateEnum {

    HOME(ElevatorPositionConstants.kHome, ArmConstants.kHome),
    CORAL_LEVEL_1(ElevatorPositionConstants.kLevel1, ArmConstants.kLowerLevelsCoralScore),
    CORAL_LEVEL_2(ElevatorPositionConstants.kLevel2, ArmConstants.kLowerLevelsCoralScore),
    CORAL_LEVEL_3(ElevatorPositionConstants.kLevel3, ArmConstants.kLowerLevelsCoralScore),
    CORAL_LEVEL_4(ElevatorPositionConstants.kLevel4, ArmConstants.kHighLevelCoralScore),
    INTAKE_CORAL(ElevatorPositionConstants.kIntakeCoralPosition, ArmConstants.kIntakeCoralPosition),
    SCORE_ALGAE_BARGE(ElevatorPositionConstants.kScoreHighAlgaePosition, ArmConstants.kHighAlgaeScore),
    INTAKE_ALGAE_23(ElevatorPositionConstants.kIntakeAlgae23Position, ArmConstants.kIntakeAlgaePosition),
    INTAKE_ALGAE_34(ElevatorPositionConstants.kIntakeAlgae34Position, ArmConstants.kIntakeAlgaePosition);
    

    public final Double elevatorValue;
    public final Double armValue;
    private StateEnum(double elevatorValue, double armValue)
    {
        this.elevatorValue = elevatorValue;
        this.armValue = armValue;
    }

}

