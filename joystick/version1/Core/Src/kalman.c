#include "stdint.h"
#include "math.h"
#include "kalman.h"
float _err_measure;
float _err_estimate;
float _q;
float _current_estimate;
float _last_estimate;
float _kalman_gain;

void SimpleKalmanFilter(float mea_e, float est_e, float q)
{
   _err_measure = mea_e;
   _err_estimate = est_e;
   _q = q;
   _current_estimate = 0.0;
   _last_estimate = 0.0;
   _kalman_gain = 0.0;
}

float updateEstimate(float mea)
{
   float denominator = _err_estimate + _err_measure;
   if (denominator == 0) {
       return _last_estimate;
   }
   _kalman_gain = _err_estimate / denominator;
   _current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
   _err_estimate = (1.0 - _kalman_gain) * _err_estimate + fabs(_last_estimate - _current_estimate) * _q;
   _last_estimate = _current_estimate;
   return _current_estimate;
}

void setMeasurementError(float mea_e)
{
   _err_measure = mea_e;
}

void setEstimateError(float est_e)
{
   _err_estimate = est_e;
}

void setProcessNoise(float q)
{
   _q = q;
}

float getKalmanGain()
{
   return _kalman_gain;
}

float getEstimateError()
{
   return _err_estimate;
}