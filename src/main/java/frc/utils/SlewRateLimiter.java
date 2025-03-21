// Source code is decompiled from a .class file using FernFlower decompiler.
package frc.utils;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;

public class SlewRateLimiter {
   private double m_positiveRateLimit;
   private double m_negativeRateLimit;
   private double m_prevVal;
   private double m_prevTime;

   public SlewRateLimiter(double positiveRateLimit, double negativeRateLimit, double initialValue) {
      this.m_positiveRateLimit = positiveRateLimit;
      this.m_negativeRateLimit = negativeRateLimit;
      this.m_prevVal = initialValue;
      this.m_prevTime = MathSharedStore.getTimestamp();
   }

   public SlewRateLimiter(double rateLimit) {
      this(rateLimit, -rateLimit, 0.0);
   }

   public double calculate(double input) {
      double currentTime = MathSharedStore.getTimestamp();
      double elapsedTime = currentTime - this.m_prevTime;
      this.m_prevVal += MathUtil.clamp(input - this.m_prevVal, this.m_negativeRateLimit * elapsedTime, this.m_positiveRateLimit * elapsedTime);
      this.m_prevTime = currentTime;
      return this.m_prevVal;
   }

   public double lastValue() {
      return this.m_prevVal;
   }

   public void reset(double value) {
      this.m_prevVal = value;
      this.m_prevTime = MathSharedStore.getTimestamp();
   }

   public void setSlewRateLimiter (double slewRateLimiter) {
        m_positiveRateLimit = slewRateLimiter;
        m_negativeRateLimit = -slewRateLimiter;
   }
}
