#include <hal_gnss/Gnss.h>

#define GYRO_XY_FRAME_ID 0x1DFFD065
#define GYRO_Z_ACEL_X_FRAME_ID 0x1DFFD165
#define ACCEL_YZ_FRAME_ID 0x1DFFD265
#define MAGN_XY_FRAME_ID 0x1DFFD365
#define MAGN_Z_ROLL_FRAME_ID 0x1DFFD465
#define PITCH_YAW_FRAME_ID 0x1DFFD565
#define LON_LAT_FRAME_ID 0x1DFFDA65
#define FIX_FRAME_ID 0x28A
#define ACC_HOR_ACC_VER_FRAME_ID 0x38A
#define HEADING_FRAME_ID 0x48A

#define GYRO_XY_READY 0x01
#define GYRO_Z_ACEL_X_READY 0x02
#define ACCEL_YZ_READY 0x04
#define MAGN_XY_READY 0x08
#define MAGN_Z_ROLL_READY 0x10
#define PITCH_YAW_READY 0x20
#define LON_LAT_READY 0x40
#define FIX_FRAME_READY 0x80
#define ACC_HOR_ACC_VER_READY 0x100
#define HEADING_FRAME_READY 0x200
#define DATA_READY 0x7F //logical OR from all above data

Gnss::Gnss(char* pchCanChannel)
{
    _pchCanChannel = pchCanChannel;
    u8FixType = 0;
    _fLongitude = 0;
    _fLatitude = 0;
    _fAccHorizontal = 0;
    _fAccVertical = 0;
    _fHeading = 0;
    _fAccHeading = 0;
    memset(_fAccelXYZ,0,3*sizeof(float));
    memset(_fGyroXYZ,0,3*sizeof(float));
    memset(_fMagnXYZ,0,3*sizeof(float));
    memset(_fRPYXYZ,0,3*sizeof(float));
    _u16DataReadiness = 0;
}

void Gnss::vParseGyroXY(can_frame Frame)
{
    _fGyroXYZ[0] = ((int32_t)Frame.data[3])<<24 | ((int32_t)Frame.data[2])<<16 |
                   ((int32_t)Frame.data[1])<<8 | ((int32_t)Frame.data[0]);
    _fGyroXYZ[0] *= (57.29/1000);
    _fGyroXYZ[1] = ((int32_t)Frame.data[7])<<24 | ((int32_t)Frame.data[6])<<16 |
                   ((int32_t)Frame.data[5])<<8 | ((int32_t)Frame.data[4]);
    _fGyroXYZ[1] *= (57.28/1000);
}

void Gnss::vParseGyroZAccelX(can_frame Frame)
{
    _fGyroXYZ[2] = ((int32_t)Frame.data[3])<<24 | ((int32_t)Frame.data[2])<<16 |
                   ((int32_t)Frame.data[1])<<8 | ((int32_t)Frame.data[0]);
    _fGyroXYZ[2] = (_fGyroXYZ[2]/1000)*57.28;
    _fAccelXYZ[0] = ((int32_t)Frame.data[7])<<24 | ((int32_t)Frame.data[6])<<16 |
                   ((int32_t)Frame.data[5])<<8 | ((int32_t)Frame.data[4]);
    _fAccelXYZ[0] /= 1000;
}
void Gnss::vParseAccelYZ(can_frame Frame)
{
    _fAccelXYZ[1] = ((int32_t)Frame.data[3])<<24 | ((int32_t)Frame.data[2])<<16 |
                   ((int32_t)Frame.data[1])<<8 | ((int32_t)Frame.data[0]);
    _fAccelXYZ[1] /= 1000;
    _fAccelXYZ[2] = ((int32_t)Frame.data[7])<<24 | ((int32_t)Frame.data[6])<<16 |
                   ((int32_t)Frame.data[5])<<8 | ((int32_t)Frame.data[4]);
    _fAccelXYZ[2] /= 1000;
}

void Gnss::vParseMagnXY(can_frame Frame)
{
    _fMagnXYZ[0] = ((int32_t)Frame.data[3])<<24 | ((int32_t)Frame.data[2])<<16 |
                   ((int32_t)Frame.data[1])<<8 | ((int32_t)Frame.data[0]);
    _fMagnXYZ[1] = ((int32_t)Frame.data[7])<<24 | ((int32_t)Frame.data[6])<<16 |
                   ((int32_t)Frame.data[5])<<8 | ((int32_t)Frame.data[4]);
}

void Gnss::vParseMagnZRoll(can_frame Frame)
{
    _fMagnXYZ[2] = ((int32_t)Frame.data[3])<<24 | ((int32_t)Frame.data[2])<<16 |
                   ((int32_t)Frame.data[1])<<8 | ((int32_t)Frame.data[0]);
    _fRPYXYZ[0] = ((int32_t)Frame.data[7])<<24 | ((int32_t)Frame.data[6])<<16 |
                  ((int32_t)Frame.data[5])<<8 | ((int32_t)Frame.data[4]);
    _fRPYXYZ[0] *= (57.29/1000);
}

void Gnss::vParsePitchYaw(can_frame Frame)
{
    _fRPYXYZ[1] = ((int32_t)Frame.data[3])<<24 | ((int32_t)Frame.data[2])<<16 |
                   ((int32_t)Frame.data[1])<<8 | ((int32_t)Frame.data[0]);
    _fRPYXYZ[1] *= (57.29/1000);
    _fRPYXYZ[2] = ((int32_t)Frame.data[7])<<24 | ((int32_t)Frame.data[6])<<16 |
                  ((int32_t)Frame.data[5])<<8 | ((int32_t)Frame.data[4]);
    _fRPYXYZ[2] *= (57.29/1000);
}

void Gnss::vParseLonLat(can_frame Frame)
{
    _fLatitude = ((int32_t)Frame.data[3])<<24 | ((int32_t)Frame.data[2])<<16 |
                  ((int32_t)Frame.data[1])<<8 | ((int32_t)Frame.data[0]);
    _fLatitude /= 10000000;
    _fLongitude = ((int32_t)Frame.data[7])<<24 | ((int32_t)Frame.data[6])<<16 |
                  ((int32_t)Frame.data[5])<<8 | ((int32_t)Frame.data[4]);
    _fLatitude /= 10000000;
}

void Gnss::vParseFix(can_frame Frame)
{
}

void Gnss::vParseAccHorAccVer(can_frame Frame)
{
}

void Gnss::vParseHeading(can_frame Frame)
{
}

int Gnss::configure()
{
    return _CanHandler.iInitialize(_pchCanChannel);
}

bool Gnss::startup()
{
    return true;
}

bool Gnss::update()
{
    can_frame frame;
    while(_CanHandler.bReceiveFrame(&frame))
    {
        switch (frame.can_id&0xFFFF)
        {
            case (GYRO_XY_FRAME_ID&0xFFFF):
                //std::cout << "A: 0x" << std::hex << GYRO_XY_FRAME_ID << std::endl;
                vParseGyroXY(frame);
                _u16DataReadiness |= GYRO_XY_READY;
                break;
            case (GYRO_Z_ACEL_X_FRAME_ID&0xFFFF):
                //std::cout << "B: 0x" << std::hex << GYRO_Z_ACEL_X_FRAME_ID << std::endl;
                vParseGyroZAccelX(frame);
                _u16DataReadiness |= GYRO_Z_ACEL_X_READY;
                break;
            case (ACCEL_YZ_FRAME_ID&0xFFFF):
                //std::cout << "C: 0x" << ACCEL_YZ_FRAME_ID << std::endl;
                vParseAccelYZ(frame);
                _u16DataReadiness |= ACCEL_YZ_READY;
                break;
            case (MAGN_XY_FRAME_ID&0xFFFF):
                //std::cout << "D: 0x" << MAGN_XY_FRAME_ID << std::endl;
                vParseMagnXY(frame);
                _u16DataReadiness |= MAGN_XY_READY;
                break;
            case (MAGN_Z_ROLL_FRAME_ID&0xFFFF):
                //std::cout << "E: 0x" << MAGN_Z_ROLL_FRAME_ID << std::endl;
                vParseMagnZRoll(frame);
                _u16DataReadiness |= MAGN_Z_ROLL_READY;
                break;
            case (PITCH_YAW_FRAME_ID&0xFFFF):
                //std::cout << "F: 0x" << PITCH_YAW_FRAME_ID << std::endl;
                vParsePitchYaw(frame);
                _u16DataReadiness |= PITCH_YAW_READY;
                break;
            case (LON_LAT_FRAME_ID&0xFFFF):
                //std::cout << "G: 0x" << LON_LAT_FRAME_ID << std::endl;
                vParseLonLat(frame);
                _u16DataReadiness |= LON_LAT_READY;
                break;
            /*case (FIX_FRAME_ID&0xFFFF):
                std::cout << "H: 0x" << FIX_FRAME_ID << std::endl;
                _u16DataReadiness |= FIX_FRAME_READY;
                vParseFix(frame);
                break;
            case (ACC_HOR_ACC_VER_FRAME_ID&0xFFFF):
                std::cout << "I: 0x" << ACC_HOR_ACC_VER_FRAME_ID << std::endl;
                vParseAccHorAccVer(frame);
                _u16DataReadiness |= ACC_HOR_ACC_VER_READY;
                break;
            case (HEADING_FRAME_ID&0xFFFF):
                std::cout << "J: 0x" << HEADING_FRAME_ID << std::endl;
                vParseHeading(frame);
                _u16DataReadiness |= HEADING_FRAME_READY;
                break; */
            default:
                break;
        }
    }
    if(_u16DataReadiness==DATA_READY)
    {
        _u16DataReadiness=0;
        return true;
    }
    else
    {
        return false;
    }
}

bool Gnss::stop()
{
    return true;
}

bool Gnss::cleanup()
{
    return true;
}