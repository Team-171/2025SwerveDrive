package frc.utils;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class GyroWrapper {
    private AHRS gyro;
    private double offset = 0;
    private boolean inverted = true;

    public GyroWrapper(AHRS gyro) {
        this.gyro = gyro;
    }

    public void setInverted(boolean isInverted) {
        inverted = isInverted;
    }

    public void setOffset(double newOffset) {
        offset = newOffset;
    }

    public double getAngle() {
        double heading = gyro.getAngle();
        // invert
        if (inverted)
            heading = -heading;

        // add offset
        heading += offset;

        while (heading > 180) {
            heading -= 360;
        }

        while (heading < -180) {
            heading += 360;
        }

        return heading;
    }

    public Rotation2d getRotation2d() {
        Rotation2d rotation2d = new Rotation2d(Math.toRadians(getAngle()));

        return rotation2d;
    }
}
