#ifndef __APP_AIM_PROTOCOL_H
#define __APP_AIM_PROTOCOL_H

#include <stdint.h>

#define AIM_FRAME_SIZE              22U
#define AIM_CRC_DATA_SIZE           20U
#define AIM_HEADER0                 0xAAU
#define AIM_HEADER1                 0x55U
#define AIM_PROTOCOL_VERSION        0x01U
#define AIM_MSG_VISION_OBSERVATION  0x01U
#define AIM_COORD_INVALID           0xFFFFU
#define AIM_IMAGE_WIDTH             640U
#define AIM_IMAGE_HEIGHT            480U

#define AIM_FLAG_RECT_VALID         (1U << 0)
#define AIM_FLAG_LASER_VALID        (1U << 1)
#define AIM_FLAG_TARGET_LOCKED      (1U << 2)
#define AIM_FLAG_MEASUREMENT_FRESH  (1U << 3)
#define AIM_FLAG_KNOWN_MASK         0x0FU

#define AIM_SEQUENCE_REBASE_TIMEOUT_MS 1000U

typedef enum
{
    AIM_TRACK_LOST      = 0,
    AIM_TRACK_ACQUIRING = 1,
    AIM_TRACK_TRACKING  = 2,
    AIM_TRACK_HOLD      = 3,
    AIM_TRACK_FAULT     = 4
} AimTrackingState_t;

typedef enum
{
    AIM_LINK_NO_SIGNAL = 0,
    AIM_LINK_FRESH,
    AIM_LINK_DEGRADED,
    AIM_LINK_STALE,
    AIM_LINK_FAULT
} AimLinkHealth_t;

typedef struct
{
    uint16_t sequence;
    uint32_t timestampMs;
    uint16_t rectX;
    uint16_t rectY;
    uint16_t laserX;
    uint16_t laserY;
    uint8_t  validFlags;
    uint8_t  trackingState;
    uint32_t rxTimeMs;
} AimObservation_t;

typedef struct
{
    uint32_t rxBytes;
    uint32_t frameCandidates;
    uint32_t validFrames;
    uint32_t crcErrors;
    uint32_t headerResyncs;
    uint32_t versionErrors;
    uint32_t typeErrors;
    uint32_t fieldErrors;
    uint32_t duplicateFrames;
    uint32_t outOfOrderFrames;
    uint32_t droppedFrames;
    uint32_t sequenceRebases;
    uint32_t parserGuardResets;
    uint8_t  selfTestPassed;
} AimProtocolStats_t;

void AimProtocol_Init(void);
void AimProtocol_Process(void);

uint16_t AimProtocol_Crc16CcittFalse(const uint8_t *data, uint16_t length);
uint8_t AimProtocol_RunSelfTest(void);

uint8_t AimProtocol_GetLatest(AimObservation_t *outObservation);
void AimProtocol_GetStats(AimProtocolStats_t *outStats);
uint8_t AimProtocol_HasValidObservation(void);

#endif
