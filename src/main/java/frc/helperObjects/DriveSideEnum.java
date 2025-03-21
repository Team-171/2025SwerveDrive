package frc.helperObjects;

import frc.robot.Constants.AutoAimConstants;

public enum DriveSideEnum {
    LEFT(AutoAimConstants.kDriveOffsetY),
    RIGHT(-AutoAimConstants.kDriveOffsetY);

    public final Double offsetY;

    private DriveSideEnum(double offsetY)
    {
        this.offsetY = offsetY;
    }
}