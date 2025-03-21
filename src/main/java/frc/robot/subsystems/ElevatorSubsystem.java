package frc.robot.subsystems;

import java.util.ArrayList;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.utils.EncoderUtils.EncoderPosition;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax m_leftLift = new SparkMax(ElevatorConstants.kFrontLiftCanID, MotorType.kBrushless);
    private final SparkMax m_rightLift = new SparkMax(ElevatorConstants.kBackLiftCanID, MotorType.kBrushless);

    private final DutyCycleEncoder encoder;
    private final Encoder relativeEncoder;
    private final EncoderPosition encoderPosition;
    private DigitalInput digitalInput;
    private DutyCycle sensor;
    private Double averageDistance;
    private ArrayList<Double> previousDistances = new ArrayList<>();

    private double holdPosition; // in rotation ticks when set for encoder

    private final PIDController pid;

    private SparkMaxConfig leftLiftConfig = new SparkMaxConfig();
    private SparkMaxConfig rightLiftConfig = new SparkMaxConfig();

    public ElevatorSubsystem() {
        digitalInput = new DigitalInput(4);
        sensor = new DutyCycle(digitalInput);

        // Encoder
        encoder = new DutyCycleEncoder(1);
        relativeEncoder = new Encoder(2, 3, false, EncodingType.k1X);
        relativeEncoder.setDistancePerPulse(0.00048828125);

        encoderPosition = new EncoderPosition(encoder.get());

        //holdPosition = encoder.get();
        //holdPosition = MathUtil.clamp(holdPosition, ElevatorConstants.kLowStopEncoder, ElevatorConstants.kHighStopEncoder);
        // distance is non-linear and not a real measurement of distance but is good enough and consistent
        averageDistance = getAverageDistance();
        SmartDashboard.putNumber("firstDistance", averageDistance);
        holdPosition = averageDistance;
        holdPosition = MathUtil.clamp(holdPosition, ElevatorConstants.kLowStopDistanceSensor, ElevatorConstants.kHighStopDistanceSensor);

        pid = new PIDController(ElevatorConstants.kElevatorP, ElevatorConstants.kElevatorI, ElevatorConstants.kElevatorD);
        pid.setIntegratorRange(0, 1);

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

    public void elevatorMove(double speed, boolean manual) {
        if(manual){
            holdPosition = averageDistance;
        }
        m_leftLift.set(speed);
        m_rightLift.set(speed);
    }

    public void stopElevator() {
        m_leftLift.stopMotor();
        m_rightLift.stopMotor();
    }

    public void holdCurrentPosition() {
        //double currentPosition = encoder.get();
        double currentPosition = averageDistance;
        if (currentPosition != 0) {

            //holdPosition = MathUtil.clamp(holdPosition, ElevatorConstants.kLowStopEncoder, ElevatorConstants.kHighStopEncoder);
            holdPosition = MathUtil.clamp(holdPosition, ElevatorConstants.kLowStopDistanceSensor, ElevatorConstants.kHighStopDistanceSensor);

            double speed = pid.calculate(currentPosition, holdPosition);
            speed = MathUtil.clamp(speed, -ElevatorConstants.kElevatorSpeed, ElevatorConstants.kElevatorSpeed);

            elevatorMove(speed, false);
        } else {
            stopElevator();
        }
    }

    public void setHoldPosition(double position) {
        holdPosition = position;
    }

    public void manualMoveArm(double speed) {
        //double currentPosition = encoder.get();
        double currentPosition = averageDistance;
        
        //if (ElevatorConstants.kLowStopEncoder >= currentPosition || ElevatorConstants.kHighStopEncoder <= currentPosition) {
        if (ElevatorConstants.kLowStopDistanceSensor >= currentPosition || ElevatorConstants.kHighStopDistanceSensor <= currentPosition) {
            speed = 0;
        }
        elevatorMove(speed, false);

        //holdPosition = encoder.get();
        holdPosition = averageDistance;
    }

    public void resetEncoder() {
        relativeEncoder.reset();
    }

    @Override
    public void periodic() {
        encoderPosition.updatePosition(encoder.get());
        // distance is non-linear and not a real measurement of distance but is good enough and consistent
        //distance = (((sensor.getHighTimeNanoseconds() / 1000) * 0.3 / 2.0) - 150.0) * 2.0;
        averageDistance = getAverageDistance();
        SmartDashboard.putNumber("averageDistance", averageDistance);
        SmartDashboard.putNumber("holdPosition", holdPosition);
    }

    private double getAverageDistance() {
        storeNewDistanceReading();

        double sum = 0;
        for (Double reading : previousDistances) {
            sum += reading;
        }
        return sum / previousDistances.size();
    }

    private void storeNewDistanceReading () {
        double distance = (((sensor.getHighTimeNanoseconds() / 1000) * 0.3 / 2.0) - 150.0) * 2.0;
        SmartDashboard.putNumber("distance", distance);
        if (previousDistances.size() > 5) {
            previousDistances.remove(0);
        }
        
        previousDistances.add(distance);
    }

}
