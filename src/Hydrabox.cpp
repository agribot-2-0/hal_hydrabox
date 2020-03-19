#include <hal_hydrabox/Hydrabox.h>

#define DEG_TO_RAD (M_PI/180)

#define GYRO_XY_FRAME_ID            0x1DFFD065
#define GYRO_Z_ACEL_X_FRAME_ID      0x1DFFD165
#define ACCEL_YZ_FRAME_ID           0x1DFFD265
#define MAGN_XY_FRAME_ID            0x1DFFD365
#define MAGN_Z_ROLL_FRAME_ID        0x1DFFD465
#define PITCH_YAW_FRAME_ID          0x1DFFD565
#define LON_LAT_FRAME_ID            0x1DFFDA65
#define FIX_FRAME_ID                0x1DFFDB65
#define ACC_HOR_ACC_VER_FRAME_ID    0x1DFFDC65
#define HEADING_FRAME_ID            0x1DFFDD65

#define BYTE_MASK 0xFFFF
#define EXPONENT_MASK 0x7f800000
#define EXPONENT_CONST 127
#define FIRST_MANTISSA_BIT 0x400000
#define FIRST_INT32 0
#define SECOND_INT32 4
#define CALCULATE_HEADING_CONST 100000

#define GAUSS_TO_TESSLA 0.0001

// Below three constants are taken from datasheet of the devices,
// for the proper ranges (accelerometer and gyroscope model - LSM6DS33,
// magnetometer - LIS3MDL). Datasheets of devices are located in Agribot_Docs.
#define G_PER_LSB (0.122)*0.001  // +/- 4g
#define DPS_PER_LSB (35)*0.001  // +/- 1000dps

#define GAUSS_PER_LSB 1/6842  // +/- 4gauss
#define TESLA_PER_LSB (GAUSS_PER_LSB)*GAUSS_TO_TESSLA

#define G_TO_M_SS 9.80655

#define FROM_MM_TO_M 0.001

// Data delivered by hydrabox is separated into two datasets,
// since it is possible, that first dataset (gyroscope, accelerometer
// magnetometr and RPY) will be sent with different frequency than second
// dataset (heading, longitude, lattitude, altitude)

// first dataset:
#define GYRO_XY_READY 0x001
#define GYRO_Z_ACEL_X_READY 0x002
#define ACCEL_YZ_READY 0x004
#define MAGN_XY_READY 0x008
#define MAGN_Z_ROLL_READY 0x010
#define PITCH_YAW_READY 0x020

// second dataset:
#define LON_LAT_READY 0x040
#define FIX_ALTITUDE_RTG_GNSS_FRAME_READY 0x080
#define ACC_HOR_ACC_VER_READY 0x100
#define HEADING_FRAME_READY 0x200

#define GYRO_OFFSET_X 0
#define GYRO_OFFSET_Y 0
#define GYRO_OFFSET_Z 0

#define ACCEL_OFFSET_X 0
#define ACCEL_OFFSET_Y 0
#define ACCEL_OFFSET_Z 0

#define OFFSET_X -6.340982145047747e-05
#define OFFSET_Y -5.059193244960625e-05
#define OFFSET_Z 8.387167599721579e-05

#define SCALE_X -2.651162977341232
#define SCALE_Y -3.264704815154654
#define SCALE_Z -0.2768223453470906

#define GAUSS_TO_TESLA 0.0001

enum eCoord {X = 0, Y = 1, Z = 2};

Hydrabox::Hydrabox(char* pchCanChannel)
{
    _pchCanChannel = pchCanChannel;
    _u16DataReadiness = 0;
}

int32_t Hydrabox::u32GetInt32(uint8_t u8StartBit, can_frame Frame)
{
    return ((int32_t)Frame.data[u8StartBit+3]) << 24
    | ((int32_t)Frame.data[u8StartBit+2]) << 16
    | ((int32_t)Frame.data[u8StartBit+1]) << 8
    | ((int32_t)Frame.data[u8StartBit]);
}

double Hydrabox::dConvertFromIEEE754(uint32_t u32NumToConvert)
{
    int iSign = (static_cast<bool>(u32NumToConvert >> 31)) ? -1 : 1;
    int32_t i32Exponent =
        ((int32_t)((u32NumToConvert&EXPONENT_MASK)>>23) - EXPONENT_CONST);
    double dMantissa = pow(2, i32Exponent);
    uint32_t u32BitMask = FIRST_MANTISSA_BIT;
    uint8_t u8BitCnt = 22;
    for (int i = i32Exponent-1; i > i32Exponent-24 ; i--)
    {
        dMantissa += (pow(2, i))*((u32BitMask&u32NumToConvert) >> u8BitCnt);
        u32BitMask = u32BitMask >> 1;
        u8BitCnt--;
    }
    dMantissa *= iSign;
    return dMantissa;
}

void Hydrabox::vParseGyroXY(can_frame Frame)
{
    _pfGyroXYZ[Y] =
        -(static_cast<float>(u32GetInt32(FIRST_INT32, Frame))*DPS_PER_LSB
        + GYRO_OFFSET_Y)*DEG_TO_RAD;
    _pfGyroXYZ[X] =
        -(static_cast<float>(u32GetInt32(SECOND_INT32, Frame))*DPS_PER_LSB
        + GYRO_OFFSET_X)*DEG_TO_RAD;
}

void Hydrabox::vParseGyroZAccelX(can_frame Frame)
{
    _pfGyroXYZ[Z] =
        -(static_cast<float>(u32GetInt32(FIRST_INT32, Frame))*DPS_PER_LSB
        + GYRO_OFFSET_Z)*DEG_TO_RAD;
    _pfAccelXYZ[Y] =
        -(static_cast<float>(u32GetInt32(SECOND_INT32, Frame))*G_PER_LSB
        *G_TO_M_SS+ACCEL_OFFSET_Y);
}

void Hydrabox::vParseAccelYZ(can_frame Frame)
{
    _pfAccelXYZ[X] =
        -(static_cast<float>(u32GetInt32(FIRST_INT32, Frame))*G_PER_LSB
        *G_TO_M_SS+ACCEL_OFFSET_X);
    _pfAccelXYZ[Z] =
        -(static_cast<float>(u32GetInt32(SECOND_INT32, Frame))*G_PER_LSB
        *G_TO_M_SS+ACCEL_OFFSET_Z);
}

void Hydrabox::vParseMagnXY(can_frame Frame)
{
    _pfMagnXYZ[Y] =
        ((static_cast<float>(u32GetInt32(FIRST_INT32, Frame))
        *GAUSS_PER_LSB)*GAUSS_TO_TESLA-OFFSET_Y)*SCALE_Y;
    _pfMagnXYZ[X] =
        ((static_cast<float>(u32GetInt32(SECOND_INT32, Frame))
        *GAUSS_PER_LSB)*GAUSS_TO_TESLA-OFFSET_X)*SCALE_X;
}

void Hydrabox::vParseMagnZRoll(can_frame Frame)
{
    _pfMagnXYZ[Z] =
        ((static_cast<float>(u32GetInt32(FIRST_INT32, Frame))*
        GAUSS_PER_LSB)*GAUSS_TO_TESLA-OFFSET_Z)*SCALE_Z;
    _pfRPYXYZ[X] = dConvertFromIEEE754(u32GetInt32(SECOND_INT32, Frame));
}

void Hydrabox::vParsePitchYaw(can_frame Frame)
{
    _pfRPYXYZ[Y] = dConvertFromIEEE754(u32GetInt32(FIRST_INT32, Frame));
    _pfRPYXYZ[Z] = dConvertFromIEEE754(u32GetInt32(SECOND_INT32, Frame));
}

void Hydrabox::vParseLonLat(can_frame Frame)
{
    _dLatitude = dConvertFromIEEE754(u32GetInt32(FIRST_INT32, Frame));
    _dLongitude = dConvertFromIEEE754(u32GetInt32(SECOND_INT32, Frame));
}

void Hydrabox::vParseHeightFix(can_frame Frame)
{
    // [4 bytes of float height in m][1 byte of fix type][1 byte of DGNSS flag]
    // [1 byte of RTK status] fix type: 0: no fix, 1: dead reckoning only,
    // 2: 2D, 3: 3D, 4:GPS - dead reckonign combined, 5: time only fix
    // RTK status - 0: node, 1: float, 2:fixed
    _dHeight = dConvertFromIEEE754(u32GetInt32(FIRST_INT32, Frame));
    _u8FixType = Frame.data[4];
    _u8DgnssFlag = Frame.data[5];
    _u8RTK = Frame.data[6];
}

void Hydrabox::vParseAccHorAccVer(can_frame Frame)
{
    _fAccHorizontal = u32GetInt32(FIRST_INT32, Frame);
    _fAccHorizontal *= FROM_MM_TO_M;
    _fAccVertical = u32GetInt32(SECOND_INT32, Frame);
    _fAccVertical *= FROM_MM_TO_M;
}

void Hydrabox::vParseHeading(can_frame Frame)
{
    _fHeading = u32GetInt32(FIRST_INT32, Frame);
    _fHeading /= CALCULATE_HEADING_CONST;
    _fAccHeading = u32GetInt32(SECOND_INT32, Frame);
    _fAccHeading /= CALCULATE_HEADING_CONST;
    _fAccHeading *= DEG_TO_RAD;
}

int Hydrabox::configure()
{
    return _CanHandler.iInitialize(_pchCanChannel);
}

bool Hydrabox::startup()
{
    return true;
}

bool Hydrabox::update()
{
    can_frame frame;
    while (_CanHandler.bReceiveFrame(&frame))
    {
        switch (frame.can_id&BYTE_MASK)
        {
            case (GYRO_XY_FRAME_ID&BYTE_MASK):
                vParseGyroXY(frame);
                _u16DataReadiness |= GYRO_XY_READY;
                break;
            case (GYRO_Z_ACEL_X_FRAME_ID&BYTE_MASK):
                vParseGyroZAccelX(frame);
                _u16DataReadiness |= GYRO_Z_ACEL_X_READY;
                break;
            case (ACCEL_YZ_FRAME_ID&BYTE_MASK):
                vParseAccelYZ(frame);
                _u16DataReadiness |= ACCEL_YZ_READY;
                break;
            case (MAGN_XY_FRAME_ID&BYTE_MASK):
                vParseMagnXY(frame);
                _u16DataReadiness |= MAGN_XY_READY;
                break;
            case (MAGN_Z_ROLL_FRAME_ID&BYTE_MASK):
                vParseMagnZRoll(frame);
                _u16DataReadiness |= MAGN_Z_ROLL_READY;
                break;
            case (PITCH_YAW_FRAME_ID&BYTE_MASK):
                vParsePitchYaw(frame);
                _u16DataReadiness |= PITCH_YAW_READY;
                break;
            case (LON_LAT_FRAME_ID&BYTE_MASK):
                vParseLonLat(frame);
                _u16DataReadiness |= LON_LAT_READY;
                break;
            case (FIX_FRAME_ID&BYTE_MASK):
                _u16DataReadiness |= FIX_ALTITUDE_RTG_GNSS_FRAME_READY;
                vParseHeightFix(frame);
                break;
            case (ACC_HOR_ACC_VER_FRAME_ID&BYTE_MASK):
                vParseAccHorAccVer(frame);
                _u16DataReadiness |= ACC_HOR_ACC_VER_READY;
                break;
            case (HEADING_FRAME_ID&BYTE_MASK):
                vParseHeading(frame);
                _u16DataReadiness |= HEADING_FRAME_READY;
                break;
            default:
                break;
        }
    }
    if ((_u16DataReadiness&FIRST_DATASET_READY) == FIRST_DATASET_READY)
    {
        return true;
    }
    if ((_u16DataReadiness&SECOND_DATASET_READY) == SECOND_DATASET_READY)
    {
        return true;
    }
    return false;
}

bool Hydrabox::stop()
{
    return true;
}

bool Hydrabox::cleanup()
{
    return true;
}

uint16_t Hydrabox::u16GetDataReadiness()
{
    return _u16DataReadiness;
}

void Hydrabox::vClearFirstDataset()
{
    _u16DataReadiness &= ~FIRST_DATASET_READY;
}

void Hydrabox::vClearSecondDataset()
{
    _u16DataReadiness &= ~SECOND_DATASET_READY;
}

float* Hydrabox::pfGetGyro()
{
    return _pfGyroXYZ;
}

float* Hydrabox::pfGetAccel()
{
    return _pfAccelXYZ;
}

float* Hydrabox::pfGetMagnetometr()
{
    return _pfMagnXYZ;
}

float* Hydrabox::pfGetRPY()
{
    return _pfRPYXYZ;
}

double Hydrabox::dGetLon()
{
    return _dLongitude;
}

double Hydrabox::dGetLat()
{
    return _dLatitude;
}

double Hydrabox::dGetHei()
{
    return _dHeight;
}

uint8_t Hydrabox::u8GetFix()
{
    return _u8FixType;
}

float Hydrabox::fGetHeading()
{
    return _fHeading;
}

float Hydrabox::fGetHeadingAcc()
{
    return _fAccHeading;
}

float Hydrabox::fGetHorizontalAcc()
{
    return _fAccHorizontal;
}

float Hydrabox::fGetVerticalAcc()
{
    return _fAccVertical;
}

uint8_t Hydrabox::u8GetGnssFlag()
{
    return _u8DgnssFlag;
}

uint8_t Hydrabox::u8GetRtk()
{
    return _u8RTK;
}

