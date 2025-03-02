package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helperObjects.DriveSide;
import frc.robot.Constants.ElevatorPositionConstants;

public class ScorePositionSubsystem extends SubsystemBase{
    
    double levelPosition;
    DriveSide side;

    public ScorePositionSubsystem() {

    }

    public void setScoreLevel(int level){
        switch (level) {
            case 1:
                levelPosition = ElevatorPositionConstants.kLevel1;
                break;
            case 2: 
                levelPosition = ElevatorPositionConstants.kLevel2;
                break;
            case 3:
                levelPosition = ElevatorPositionConstants.kLevel3;
                break;
            case 4:
                levelPosition = ElevatorPositionConstants.kLevel4;
                break;        
            default:
                levelPosition = ElevatorPositionConstants.kLevel1;
                break;
        }
    }

    public void setScoreLeftOrRight(int povPosition){
        switch (povPosition) {
            case 90:
                side = DriveSide.RIGHT;
                break;
            case 270:
                side = DriveSide.LEFT;
                break;
            default:
                break;
        }
    }

    public double returnLevelHoldPosition(){
        return levelPosition;
    }

    
}
