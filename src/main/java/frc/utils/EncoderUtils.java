package frc.utils;

public class EncoderUtils {
    public static class EncoderPosition {
        private double rollover;
        private double encoderPosition;
        private boolean closeToTop;
        private boolean closeToBottom;

        public EncoderPosition(double encoderPosition) {
            this.encoderPosition = encoderPosition;
            this.rollover = 0;
            closeToBottom = false;
            closeToTop = true;
            isClose();
        }

        public void updatePosition(double encoderPosition) {
            if (closeToTop && encoderPosition < .1) {
                rollover += 1;
            }
            if (closeToBottom && encoderPosition > 0.9){
                rollover -= 1;
            }
            isClose();
        }

        private void isClose() {
            if (this.encoderPosition >= 0.9) {
                closeToTop = true;
            } else {
                closeToTop = false;
            }
            if (this.encoderPosition <= 0.1) {
                closeToBottom = true;
            } else {
                closeToBottom = false;
            }
        }

        public double getCurrentRollover() {
            return rollover;
        }

        public double getPositionWithRollover(double encoderPosition){
            double position = encoderPosition + rollover;
            return position;
        }
    }
}
