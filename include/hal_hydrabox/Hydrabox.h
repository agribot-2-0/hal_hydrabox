#pragma once

#include <agribot_common/AgribotComponent.h>
#include <agribot_common/Can.h>

#define FIRST_DATASET_READY 0x7F
#define SECOND_DATASET_READY 0x380

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

    float _pfAccelXYZ[3]; 
    float _pfGyroXYZ[3];
    float _pfMagnXYZ[3];
    float _pfRPYXYZ[3];
    uint16_t _u16DataReadiness;

    uint32_t u32GetInt32(uint8_t u8StartBit, can_frame Frame);
    float fConvertFromIEEE(uint32_t u32NumToConvert);

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

    uint16_t u16GetDataReadiness();
    void vClearFirstDataset();
    void vClearSecondDataset();

    float* pfGetGyro();
    float* pfAccel();
    float* pfMagnetometr();
    float* pfGetRPY();
};
