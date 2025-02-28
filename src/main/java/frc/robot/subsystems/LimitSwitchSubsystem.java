package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimitSwitchSubsystem extends SubsystemBase {

    public LimitSwitchSubsystem() {

    }

    public void controllerRumble(XboxController controller, DigitalInput limitSwitch){
        if(limitSwitch.get()){
            controller.setRumble(RumbleType.kBothRumble, .75);
        }else{
            controller.setRumble(RumbleType.kBothRumble, 0);
        }

        SmartDashboard.putBoolean("Limit Switch", limitSwitch.get());
    }

    @Override
    public void periodic() {
    }
}
