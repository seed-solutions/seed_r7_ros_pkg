#pragma once

#include "ms_data.hpp"
#include "cmd_data.hpp"

void createFirmwareVersion(MsRecvRaw *recvd, AeroSendRaw *sendd);

void createGetposResp(MSData *data, MsRecvRaw *recvd, AeroSendRaw *sendd);

void createGetvolResp(MSData *data, MsRecvRaw *recvd, AeroSendRaw *sendd);

void createGetEEPROMResp(MSData *data, MsRecvRaw *recvd, AeroSendRaw *sendd);

void createSetEEPROMResp(MSData *data, MsRecvRaw *recvd,AeroSendRaw *sendd);

void createMovePosResp(MSData *data, MsRecvRaw *recvd,AeroSendRaw *sendd);

void createMoveVelResp(MSData *data, MsRecvRaw *recvd,AeroSendRaw *sendd);

void execServoCmd(MSData *data, MsRecvRaw *recvd);

void createUpperBodyMsData(uint8_t *data);

void createUpperBodySendNoData(uint8_t *data);

