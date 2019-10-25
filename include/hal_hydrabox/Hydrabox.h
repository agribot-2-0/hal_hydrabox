#pragma once

#include <agribot_common/AgribotComponent.h>
#include <agribot_common/Can.h>

class Hydrabox: public AgribotComponent
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
    Hydrabox(char* pchCanChannel);

    void vParseGyroXY(can_frame Frame);
    void vParseGyroZAccelX(can_frame Frame);
    void vParseAccelYZ(can_frame Frame);
    void vParseMagnXY(can_frame Frame);
    void vParseMagnZRoll(can_frame Frame);
    void vParsePitchYaw(can_frame Frame);
    void vParseLonLat(can_frame Frame);
    void vParseFix(can_frame Frame);
    void vParseAccHorAccVer(can_frame Frame);
    void vParseHeading(can_frame Frame);

    virtual int configure();
    virtual bool startup();
    virtual bool update();
    virtual bool stop();
    virtual bool cleanup();
};
