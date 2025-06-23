#ifndef KALMAN_H_
#define KALMAN_H_
 // class SimpleKalmanFilter
 void SimpleKalmanFilter(float mea_e, float est_e, float q);
   float updateEstimate(float mea);
   void setMeasurementError(float mea_e);
   void setEstimateError(float est_e);
   void setProcessNoise(float q);
   float getKalmanGain();
   float getEstimateError();
 #endif
 
 /*
 #include "kalman.h"

main()
{
    SimpleKalmanFilter(1, 2, 0.001); // Kh?i t?o cho kalman
    while(1)
    {
        char u1 = ReadADC();
        char UKalman = (uint8_t)updateEstimate((float)u1);
    }
}
 */