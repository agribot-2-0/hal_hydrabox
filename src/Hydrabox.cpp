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
#define GYRO_XY_READY 0x001
#define GYRO_Z_ACEL_X_READY 0x002
#define ACCEL_YZ_READY 0x004
#define MAGN_XY_READY 0x008
#define MAGN_Z_ROLL_READY 0x010
#define PITCH_YAW_READY 0x020

//second dataset:
#define LON_LAT_READY 0x040
#define FIX_FRAME_READY 0x080
#define ACC_HOR_ACC_VER_READY 0x100
#define HEADING_FRAME_READY 0x200

Hydrabox::Hydrabox(char* pchCanChannel)
{
    _pchCanChannel = pchCanChannel;
    u8FixType = 0;
    _dLongitude = 0;
    _dLatitude = 0;
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

double Hydrabox::dConvertFromIEEE(uint32_t u32NumToConvert)
{
    int iSing = ((bool)(u32NumToConvert >> 31)) ? -1 : 1;
    int32_t i32Exponent = ((int32_t)((u32NumToConvert&0x7f800000)>>23) - 127);
    double dMantissa = pow(2,i32Exponent);
    uint32_t u32BitMask = 0x400000;
    uint8_t u8BitCnt = 22;

    for (int i = i32Exponent-1; i > i32Exponent-24 ; i--)
    {
        dMantissa += (pow(2,i))*((u32BitMask&u32NumToConvert)>>u8BitCnt);
        u32BitMask = u32BitMask >> 1;
        u8BitCnt--;
    }
}

void Hydrabox::vParseGyroXY(can_frame Frame)
{
    _pfGyroXYZ[0] = u32GetInt32(0,Frame);
    _pfGyroXYZ[1] = u32GetInt32(4,Frame);
}

void Hydrabox::vParseGyroZAccelX(can_frame Frame)
{
    _pfGyroXYZ[2] = u32GetInt32(0,Frame);
    _pfAccelXYZ[0] = u32GetInt32(4,Frame);
}

void Hydrabox::vParseAccelYZ(can_frame Frame)
{
    _pfAccelXYZ[1] = u32GetInt32(0,Frame);
    _pfAccelXYZ[2] = u32GetInt32(4,Frame);
}

void Hydrabox::vParseMagnXY(can_frame Frame)
{
    _pfMagnXYZ[0] = u32GetInt32(0,Frame);
    _pfMagnXYZ[1] = u32GetInt32(4,Frame);
}

void Hydrabox::vParseMagnZRoll(can_frame Frame)
{
    _pfMagnXYZ[2] = u32GetInt32(0,Frame);
    _pfRPYXYZ[0] = dConvertFromIEEE(u32GetInt32(4,Frame));
}

void Hydrabox::vParsePitchYaw(can_frame Frame)
{
    _pfRPYXYZ[1] = dConvertFromIEEE(u32GetInt32(0,Frame));
    _pfRPYXYZ[2] = dConvertFromIEEE(u32GetInt32(4,Frame));
}

void Hydrabox::vParseLonLat(can_frame Frame)
{
    _dLatitude = dConvertFromIEEE(u32GetInt32(0,Frame));
    _dLongitude = dConvertFromIEEE(u32GetInt32(4,Frame));
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
