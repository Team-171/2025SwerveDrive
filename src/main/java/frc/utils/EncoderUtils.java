package frc.utils;

public class EncoderUtils {
    public static class EncoderPosition {
        private final double startingPosition;
        private double encoderPosition;
        public static boolean firstRun;

        public EncoderPosition(double position) {
            startingPosition = position;
            encoderPosition = position;
            firstRun = true;
        }

        public void updatePosition(double position) {
            
        }

        public double getStartingPosition(){
            return startingPosition;
        }

        public double getPosition(double position){
            return encoderPosition;
        }

        public void completeFirstRun(){
            firstRun = false;
        }
    }
}
