#pragma once

#include <agribot_common/AgribotComponent.h>
#include <agribot_common/Can.h>
#include <cmath>

#define FIRST_DATASET_READY 0x3F
#define SECOND_DATASET_READY 0x3C0

class Hydrabox: public AgribotComponent
{
private:
    Can _CanHandler;
    char* _pchCanChannel;
    uint16_t _u16DataReadiness;

    uint8_t _u8FixType;
    uint8_t _u8DgnssFlag;
    uint8_t _u8RTK;
    double _dLongitude;
    double _dLatitude;
    double _dHeight;
    float _fAccHorizontal;
    float _fAccVertical;
    float _fHeading;
    float _fAccHeading;

    float _pfAccelXYZ[3]; 
    float _pfGyroXYZ[3];
    float _pfMagnXYZ[3];
    float _pfRPYXYZ[3];

    // [magn_offset_x magn_offset_y magn_offset_z
    //  magn_scale_x magn_scale_y magn_scale_z gyro_offset_x gyro_offset_y gyro_offset_z
    //  accel_offset_x accel_offset_y accel_offset_z]
    float pfOffsets[12];

    int32_t u32GetInt32(uint8_t u8StartBit, can_frame Frame);
    
    //arg: uint32_t u32NumToConvert - 32 bit encoded float in IEEE 754 standard
    //return: decoded number
    double dConvertFromIEEE754(uint32_t u32NumToConvert);
public:
    Hydrabox(char* pchCanChannel);
    void vInitOffsets(char** fpParameters);

    void vParseGyroXY(can_frame Frame);
    void vParseGyroZAccelX(can_frame Frame);
    void vParseAccelYZ(can_frame Frame);
    void vParseMagnXY(can_frame Frame);
    void vParseMagnZRoll(can_frame Frame);
    void vParsePitchYaw(can_frame Frame);
    void vParseLonLat(can_frame Frame);
    void vParseHeightFix(can_frame Frame);
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
    float* pfGetAccel();
    float* pfGetMagnetometr();
    float* pfGetRPY();
    double dGetLon();
    double dGetLat();
    double dGetHei();
    uint8_t u8GetFix();
    float fGetHeading();
    float fGetHeadingAcc();
    float fGetHorizontalAcc();
    float fGetVerticalAcc();
    uint8_t u8GetGnssFlag();
    uint8_t u8GetRtk();
};
