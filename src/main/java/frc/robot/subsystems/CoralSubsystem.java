package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.CoralConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSubsystem extends SubsystemBase{

    private final SparkMax m_rollerTop = new SparkMax(CoralConstants.kCoralMotorCanID, MotorType.kBrushless);
    private DigitalInput limitSwitch = new DigitalInput(CoralConstants.kLimitSwitchChannel);

    public CoralSubsystem() {

        SparkMaxConfig topMaxConfig = new SparkMaxConfig();
        
        topMaxConfig
            .smartCurrentLimit(20)
            .idleMode(IdleMode.kBrake)
            .closedLoopRampRate(0.125);

        m_rollerTop.configure(topMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
 
    /**
     * Spins the motors to move the coral in
     * @param speed Speed at which to spin the motors
     */
    public void intakeCoral(double speed) {
        SmartDashboard.putBoolean("coralLimitSwitch", limitSwitch.get());
        if (limitSwitch.get() == false) {
            m_rollerTop.set(speed);    
        } 
        else {
            m_rollerTop.set(0);
        }
       
    }

    public void outputCoral(double speed) {
        m_rollerTop.set(speed);
    }

    public void stopMotion() {
        m_rollerTop.stopMotor();
    }
    
}
