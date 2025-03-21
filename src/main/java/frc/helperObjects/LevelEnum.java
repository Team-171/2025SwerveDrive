package frc.helperObjects;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorPositionConstants;

public enum LevelEnum {
    LEVEL_1(ElevatorPositionConstants.kLevel1, ArmConstants.kLowerLevelsCoralScore),
    LEVEL_2(ElevatorPositionConstants.kLevel2, ArmConstants.kLowerLevelsCoralScore),
    LEVEL_3(ElevatorPositionConstants.kLevel3, ArmConstants.kLowerLevelsCoralScore),
    LEVEL_4(ElevatorPositionConstants.kLevel4, ArmConstants.kHighLevelCoralScore);

    public final Double elevatorValue;
    public final Double armValue;
    private LevelEnum(double elevatorValue, double armValue)
    {
        this.elevatorValue = elevatorValue;
        this.armValue = armValue;
    }
}
