package frc.robot.subsystems;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.Unit;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax m_leftLift = new SparkMax(ElevatorConstants.kLeftLiftCanID, MotorType.kBrushless);
    private final SparkMax m_rightLift = new SparkMax(ElevatorConstants.kRightLiftCanID, MotorType.kBrushless);
    // private final DutyCycleEncoder m_winchEncoder = new
    // DutyCycleEncoder(ElevatorConstants.kElevatorPulleyChannelID, 4, 2.0);
    private final DutyCycleEncoder encoder;
    private final Rev2mDistanceSensor distanceSensor;

    private double holdPosition; // in mm

    private final PIDController pid;

    public ElevatorSubsystem() {

        // Elevator Motors
        SparkMaxConfig leftLiftConfig = new SparkMaxConfig();
        SparkMaxConfig rightLiftConfig = new SparkMaxConfig();

        // Encoder
        encoder = new DutyCycleEncoder(0);

        distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);
        distanceSensor.setDistanceUnits(Unit.kMillimeters);

        holdPosition = distanceSensor.getRange();

        pid = new PIDController(1, 0, 0);

        leftLiftConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(ElevatorConstants.kElevatorMotorCurrentLimit)
                .closedLoopRampRate(ElevatorConstants.kElevatorMotorRampRate);
        rightLiftConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(ElevatorConstants.kElevatorMotorCurrentLimit)
                .closedLoopRampRate(ElevatorConstants.kElevatorMotorRampRate)
                .inverted(true);

        m_leftLift.configure(leftLiftConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        m_rightLift.configure(rightLiftConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public void elevatorMove(double speed) {
        m_leftLift.set(speed);
        m_rightLift.set(speed);
    }

    public void stopElevator() {
        m_leftLift.stopMotor();
        m_rightLift.stopMotor();
    }

    public void holdCurrentPosition() {
        double currentPosition = distanceSensor.getRange();
        if (currentPosition != 0) {

            holdPosition = MathUtil.clamp(holdPosition, ElevatorConstants.kLowStop, ElevatorConstants.kHighStop);

            double speed = pid.calculate(currentPosition, holdPosition);
            speed = MathUtil.clamp(speed, -ElevatorConstants.kElevatorSpeed, ElevatorConstants.kElevatorSpeed);

            elevatorMove(speed);
        } else {
            stopElevator();
        }
    }

    public void setHoldPosition(double position) {
        holdPosition = position;
    }

    public void manualMoveArm(double speed) {
        double currentPosition = distanceSensor.getRange();
        if(ElevatorConstants.kLowStop >= currentPosition || ElevatorConstants.kHighStop <= currentPosition){
            speed = 0;
        }
        elevatorMove(speed);

        holdPosition = distanceSensor.getRange();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("encoder", encoder.get());
        SmartDashboard.putNumber("distanceSensor", distanceSensor.getRange());
    }
}
