#include <hal_hydrabox/Hydrabox.h>

#define GYRO_XY_FRAME_ID 			0x1DFFD065
#define GYRO_Z_ACEL_X_FRAME_ID 		0x1DFFD165
#define ACCEL_YZ_FRAME_ID 			0x1DFFD265
#define MAGN_XY_FRAME_ID 			0x1DFFD365
#define MAGN_Z_ROLL_FRAME_ID 		0x1DFFD465
#define PITCH_YAW_FRAME_ID 			0x1DFFD565
#define LON_LAT_FRAME_ID 			0x1DFFDA65
#define FIX_FRAME_ID 				0x1DFFDB65
#define ACC_HOR_ACC_VER_FRAME_ID 	0x1DFFDC65
#define HEADING_FRAME_ID 			0x1DFFDD65

//first dataset:
#define GYRO_XY_READY 0x01
#define GYRO_Z_ACEL_X_READY 0x02
#define ACCEL_YZ_READY 0x04
#define MAGN_XY_READY 0x08
#define MAGN_Z_ROLL_READY 0x10
#define PITCH_YAW_READY 0x20
#define LON_LAT_READY 0x40

//second dataset:
#define FIX_FRAME_READY 0x80
#define ACC_HOR_ACC_VER_READY 0x100
#define HEADING_FRAME_READY 0x200

Hydrabox::Hydrabox(char* pchCanChannel)
{
    _pchCanChannel = pchCanChannel;
    u8FixType = 0;
    _fLongitude = 0;
    _fLatitude = 0;
    _fAccHorizontal = 0;
    _fAccVertical = 0;
    _fHeading = 0;
    _fAccHeading = 0;
    memset(_pfAccelXYZ,0,3*sizeof(float));
    memset(_pfGyroXYZ,0,3*sizeof(float));
    memset(_pfMagnXYZ,0,3*sizeof(float));
    memset(_pfRPYXYZ,0,3*sizeof(float));
    _u16DataReadiness = 0;
}

uint32_t Hydrabox::u32GetInt32(uint8_t u8StartBit, can_frame Frame)
{
    return ((uint32_t)Frame.data[u8StartBit+3])<<24 | ((uint32_t)Frame.data[u8StartBit+2])<<16 |
    ((uint32_t)Frame.data[u8StartBit+1])<<8 | ((uint32_t)Frame.data[u8StartBit]);
}

float Hydrabox::fConvertFromIEEE(uint32_t u32NumToConvert)
{
    int iSing = ((bool)u32NumToConvert >> 31) ? -1 : 1;
    int iExponent = (u32NumToConvert&0x7f800000) - 127;
    float fMantissa = 0;
}

void Hydrabox::vParseGyroXY(can_frame Frame)
{
    _pfGyroXYZ[0] = u32GetInt32(0,Frame);
    _pfGyroXYZ[0] *= (57.29/1000);
    _pfGyroXYZ[1] = u32GetInt32(4,Frame);
    _pfGyroXYZ[1] *= (57.28/1000);
}

void Hydrabox::vParseGyroZAccelX(can_frame Frame)
{
    _pfGyroXYZ[2] = u32GetInt32(0,Frame);
    _pfGyroXYZ[2] *= (57.29/1000);
    _pfAccelXYZ[0] = u32GetInt32(4,Frame);
    _pfAccelXYZ[0] /= 1000;
}
void Hydrabox::vParseAccelYZ(can_frame Frame)
{
    _pfAccelXYZ[1] = u32GetInt32(0,Frame);
    _pfAccelXYZ[1] /= 1000;
    _pfAccelXYZ[2] = u32GetInt32(4,Frame);
    _pfAccelXYZ[2] /= 1000;
}

void Hydrabox::vParseMagnXY(can_frame Frame)
{
    _pfMagnXYZ[0] = u32GetInt32(0,Frame);
    _pfMagnXYZ[1] = u32GetInt32(4,Frame);
}

void Hydrabox::vParseMagnZRoll(can_frame Frame)
{
    _pfMagnXYZ[2] = u32GetInt32(0,Frame);
    _pfRPYXYZ[0] = u32GetInt32(4,Frame);
    _pfRPYXYZ[0] *= (57.29/1000);
}

void Hydrabox::vParsePitchYaw(can_frame Frame)
{
    _pfRPYXYZ[1] = u32GetInt32(0,Frame);
    _pfRPYXYZ[1] *= (57.29/1000);
    _pfRPYXYZ[2] = u32GetInt32(4,Frame);
    _pfRPYXYZ[2] *= (57.29/1000);
}

void Hydrabox::vParseLonLat(can_frame Frame)
{
    _fLatitude = u32GetInt32(0,Frame);
    _fLongitude = u32GetInt32(4,Frame);
    std::cout << "0x";
    std::cout << std::hex << (uint32_t)_fLatitude; 
    std::cout << "   " << "0x";
    std::cout << std::hex << (uint32_t)_fLongitude; 
    std::cout << std::endl;
}

void Hydrabox::vParseFix(can_frame Frame)
{
	//[4 bytes of float height in m][1 byte of fix type][1 byte of DGNSS flag][1 byte of RTK status]
	//fix type: 0: no fix, 1: dead reckoning only, 2: 2D, 3: 3D, 4:GPS - dead reckonign combined, 5: time only fix
	//RTK status - 0: node, 1: float, 2:fixed
    u8FixType = Frame.data[4];
}

void Hydrabox::vParseAccHorAccVer(can_frame Frame)
{
    _fAccHorizontal = u32GetInt32(0,Frame);
    _fAccVertical = u32GetInt32(4,Frame);
}

void Hydrabox::vParseHeading(can_frame Frame)
{
    _fHeading = u32GetInt32(0,Frame);
    _fHeading /= 100000;
    _fAccHeading = u32GetInt32(4,Frame);
    _fAccHeading /= 100000;
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
            case (FIX_FRAME_ID&0xFFFF):
                //std::cout << "H: 0x" << FIX_FRAME_ID << std::endl;
                _u16DataReadiness |= FIX_FRAME_READY;
                vParseFix(frame);
                break;
            case (ACC_HOR_ACC_VER_FRAME_ID&0xFFFF):
                //std::cout << "I: 0x" << ACC_HOR_ACC_VER_FRAME_ID << std::endl;
                vParseAccHorAccVer(frame);
                _u16DataReadiness |= ACC_HOR_ACC_VER_READY;
                break;
            case (HEADING_FRAME_ID&0xFFFF):
                //std::cout << "J: 0x" << HEADING_FRAME_ID << std::endl;
                vParseHeading(frame);
                _u16DataReadiness |= HEADING_FRAME_READY;
                break;
            default:
                break;
        }
    }
    if((_u16DataReadiness&FIRST_DATASET_READY) == FIRST_DATASET_READY)
    {
        return true;
    }
    if((_u16DataReadiness&SECOND_DATASET_READY) == SECOND_DATASET_READY)
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

float* Hydrabox::pfAccel()
{
    return _pfAccelXYZ;
}

float* Hydrabox::pfMagnetometr()
{
    return _pfMagnXYZ;
}

float* Hydrabox::pfGetRPY()
{
    return _pfRPYXYZ;
}
