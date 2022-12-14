#ifndef COOK_h
#define COOK_h

#include <Arduino.h>
#include <DallasTemperature.h>
#include <SimpleKalmanFilter.h>


class Cook {
  private:
    int burnerPin;
    int burnerStatus;
    OneWire *oneWire;
    DallasTemperature *sensors;
    DeviceAddress tempSensor;
    SimpleKalmanFilter *kalmanFilter;
    
    int alarmPin;
    String alarmMessage;

    double goalTemp;
    double currentTemp;
    double waveCrest;
    // slope indicates how the temperature is evolving
    // = 0  --> stable, not moving
    // > 0  --> temperature increasing
    // < 0  --> decreasing
    int slope;

    long goalTimeInSeconds;
    long currentTimeInSeconds;    
    
  public:
    static const double DefaultGoalTemp = 92.0;
    static const long   DefaultGoalTime = 6 * 60 * 60;  // 6 hours
    
    Cook(int burner, int alarm, double cookTemp = DefaultGoalTemp, long timeInSecs = DefaultGoalTime);

    void refresh();

    void adjustGoalTemp( double delta );
    void adjustGoalTime( int hours, int minutes = 0 );
    
    int getSlope();
    double getGoalTemp();
    double getCurrentTemp();
    long getGoalTimeInSecs();
    long getCurrentTimeInSecs();

    void setAlarm(String msg);
    bool alarm();
    String getAlarmMessage();
    void turnOffAlarm();
};

#endif  // COOK_h
