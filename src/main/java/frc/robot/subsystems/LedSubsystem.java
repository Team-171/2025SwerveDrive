package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {

    Spark ledStrip = new Spark(2);

    public LedSubsystem() {

    }

    public void limitSwitchLed(DigitalInput limitSwitch){
        if(limitSwitch.get()){
            ledStrip.set(-0.49);
        }else{
            ledStrip.set(0.51);
        }
    }

    @Override
    public void periodic() {

    }
}
