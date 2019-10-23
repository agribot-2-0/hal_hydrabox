#pragma once

#include <agribot_common/AgribotComponent.h>
#include <agribot_common/Can.h>

class Gnss: public AgribotComponent
{
private:
    Can _CanHandler;
    char* _pchCanChannel;

    uint8_t u8FixType; 

    float _fLongitude;
    float _fLatitude;

    float _fAccHorizontal;
    float _fAccVertical;

    float _fHeading;
    float _fAccHeading;

    float _fAccelXYZ[3]; 
    float _fGyroXYZ[3];
    float _fMagnXYZ[3];
    float _fRPYXYZ[3];
    uint16_t _u16DataReadiness;
public:
    Gnss(char* pchCanChannel);
    virtual int configure();
    virtual bool startup();
    virtual bool update();
    virtual bool stop();
    virtual bool cleanup();
};
