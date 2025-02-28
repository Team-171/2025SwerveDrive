package frc.utils;

import frc.helperObjects.XYCoordinates;

public class PolarUtils {

    /**
     * Translates polar coordinates to cartesian coordinates
     * @param radius distance from center
     * @param angle direction of coordinate (degrees)
     * @return XYCoordinates of the translated destination
     */
    public static XYCoordinates polarToCartesian (double radius, double angle) {
        XYCoordinates xyCoordinates = new XYCoordinates();
        double radianAngle = Math.toRadians(angle);
        xyCoordinates.setXCoord(radianAngle * Math.cos(angle)); // x coordinate
        xyCoordinates.setYCoord(radianAngle * Math.sin(angle));;  // y coordinate
        return xyCoordinates;
    }
}
