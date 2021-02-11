/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <agribot_common/AgribotComponent.h>
#include <agribot_common/Can.h>
#include <cmath>

#define FIRST_DATASET_READY 0x3F
#define SECOND_DATASET_READY 0x3C0

enum class HydraboxFixType
{
  no_fix = 0,
  dead_reckoning_only = 1,
  two_d = 2,
  three_d = 3,
  gps_dead_reckoning_combined = 4,
  time_only_fix = 5
};

enum class HydraboxRtkStatus
{
  node = 0,
  floating = 1,
  fixed = 2
};

class Hydrabox : public AgribotComponent
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
  double _fAccHorizontal;
  double _fAccVertical;
  double _fHeading;
  double _fAccHeading;

  double _pfAccelXYZ[3];
  double _pfGyroXYZ[3];
  double _pfMagnXYZ[3];
  double _pfRPYXYZ[3];

  // Accelerometer, gyroscope and magnetometer bias
  double accel_offset_x_;
  double accel_offset_y_;
  double accel_offset_z_;

  double gyro_offset_x_;
  double gyro_offset_y_;
  double gyro_offset_z_;

  double mag_offset_x_;
  double mag_offset_y_;
  double mag_offset_z_;

  double mag_scale_x_;
  double mag_scale_y_;
  double mag_scale_z_;

  int32_t u32GetInt32(uint8_t u8StartBit, can_frame Frame);

  // arg: uint32_t u32NumToConvert - 32 bit encoded float in IEEE 754 standard
  // return: decoded number
  double dConvertFromIEEE754(uint32_t u32NumToConvert);

 public:
  explicit Hydrabox(char* pchCanChannel);

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

  void setAccelOffset(double x, double y, double z);
  void setGyroOffset(double x, double y, double z);
  void setMagOffset(double x, double y, double z);
  void setMagScale(double x, double y, double z);

  uint16_t u16GetDataReadiness();
  void vClearFirstDataset();
  void vClearSecondDataset();

  double* pfGetGyro();
  double* pfGetAccel();
  double* pfGetMagnetometr();
  double* pfGetRPY();
  double dGetLon();
  double dGetLat();
  double dGetHei();
  uint8_t u8GetFix();
  double fGetHeading();
  double fGetHeadingAcc();
  double fGetHorizontalAcc();
  double fGetVerticalAcc();
  uint8_t u8GetGnssFlag();
  uint8_t u8GetRtk();
};
