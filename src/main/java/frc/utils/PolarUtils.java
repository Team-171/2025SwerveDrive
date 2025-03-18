package frc.utils;

import frc.helperObjects.XYCoordinates;

public class PolarUtils {

    /**
     * Translates polar coordinates to cartesian coordinates
     * @param radius distance from center
     * @param angle direction of coordinate (radians)
     * @return XYCoordinates of the translated destination
     */
    public static XYCoordinates polarToCartesian (double radius, double angle) {
        XYCoordinates xyCoordinates = new XYCoordinates();
        xyCoordinates.setXCoord(angle * Math.cos(angle)); // x coordinate
        xyCoordinates.setYCoord(angle * Math.sin(angle));;  // y coordinate
        return xyCoordinates;
    }
}
