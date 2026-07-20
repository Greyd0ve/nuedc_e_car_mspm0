#include "app_aim_protocol.h"
#include "K230Uart.h"
#include "Timer.h"
#include "cmsis_compiler.h"

typedef enum
{
    AIM_PARSE_WAIT_HEADER0 = 0,
    AIM_PARSE_WAIT_HEADER1,
    AIM_PARSE_COLLECT
} AimParseState_t;

static AimObservation_t s_latest;
static AimProtocolStats_t s_stats;

static uint8_t  s_frame[AIM_FRAME_SIZE];
static uint8_t  s_frameIndex;
static uint8_t  s_parseState;
static uint8_t  s_hasLatest;
static uint8_t  s_hasLastSequence;
static uint16_t s_lastSequence;

static uint16_t AimProtocol_ReadLe16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint32_t AimProtocol_ReadLe32(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

uint16_t AimProtocol_Crc16CcittFalse(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;

    while (length > 0U)
    {
        crc ^= ((uint16_t)(*data) << 8);
        data++;
        length--;

        for (i = 0U; i < 8U; i++)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            }
            else
            {
                crc = (uint16_t)(crc << 1);
            }
        }
    }

    return crc;
}

static uint8_t AimProtocol_IsCoordValid(uint16_t coord)
{
    return (uint8_t)((coord < AIM_IMAGE_WIDTH || coord == AIM_COORD_INVALID) ? 1U : 0U);
}

static uint8_t AimProtocol_IsRectCoordInBound(uint16_t coord)
{
    return (uint8_t)((coord < AIM_IMAGE_WIDTH) ? 1U : 0U);
}

static uint8_t AimProtocol_ValidateFrame(void)
{
    uint16_t version;
    uint16_t msgType;
    uint16_t validFlags;
    uint16_t trackingState;
    uint16_t rectX, rectY, laserX, laserY;
    uint16_t calcCrc, recvCrc;

    calcCrc = AimProtocol_Crc16CcittFalse(s_frame, AIM_CRC_DATA_SIZE);
    recvCrc = AimProtocol_ReadLe16(&s_frame[20]);
    if (calcCrc != recvCrc)
    {
        s_stats.crcErrors++;
        return 0U;
    }

    version = s_frame[2];
    if (version != AIM_PROTOCOL_VERSION)
    {
        s_stats.versionErrors++;
        return 0U;
    }

    msgType = s_frame[3];
    if (msgType != AIM_MSG_VISION_OBSERVATION)
    {
        s_stats.typeErrors++;
        return 0U;
    }

    validFlags = s_frame[18];
    if ((validFlags & ~AIM_FLAG_KNOWN_MASK) != 0U)
    {
        s_stats.fieldErrors++;
        return 0U;
    }

    trackingState = s_frame[19];
    if (trackingState > 4U)
    {
        s_stats.fieldErrors++;
        return 0U;
    }

    rectX  = AimProtocol_ReadLe16(&s_frame[10]);
    rectY  = AimProtocol_ReadLe16(&s_frame[12]);
    laserX = AimProtocol_ReadLe16(&s_frame[14]);
    laserY = AimProtocol_ReadLe16(&s_frame[16]);

    if (!AimProtocol_IsCoordValid(rectX) || !AimProtocol_IsCoordValid(rectY) ||
        !AimProtocol_IsCoordValid(laserX) || !AimProtocol_IsCoordValid(laserY))
    {
        s_stats.fieldErrors++;
        return 0U;
    }

    if ((validFlags & AIM_FLAG_RECT_VALID) != 0U)
    {
        if (!AimProtocol_IsRectCoordInBound(rectX) || !AimProtocol_IsRectCoordInBound(rectY))
        {
            s_stats.fieldErrors++;
            return 0U;
        }
    }
    else
    {
        if (rectX != AIM_COORD_INVALID || rectY != AIM_COORD_INVALID)
        {
            s_stats.fieldErrors++;
            return 0U;
        }
    }

    if ((validFlags & AIM_FLAG_LASER_VALID) != 0U)
    {
        if (!AimProtocol_IsRectCoordInBound(laserX) || !AimProtocol_IsRectCoordInBound(laserY))
        {
            s_stats.fieldErrors++;
            return 0U;
        }
    }
    else
    {
        if (laserX != AIM_COORD_INVALID || laserY != AIM_COORD_INVALID)
        {
            s_stats.fieldErrors++;
            return 0U;
        }
    }

    return 1U;
}

static void AimProtocol_AcceptFrame(void)
{
    uint16_t sequence;
    uint16_t delta;

    sequence = AimProtocol_ReadLe16(&s_frame[4]);

    if (!s_hasLastSequence)
    {
        s_hasLastSequence = 1U;
        s_lastSequence    = sequence;
    }
    else
    {
        delta = (uint16_t)(sequence - s_lastSequence);

        if (delta == 0U)
        {
            s_stats.duplicateFrames++;
            return;
        }

        if (delta >= 0x8000U)
        {
            s_stats.outOfOrderFrames++;
            return;
        }

        if (delta > 1U)
        {
            s_stats.droppedFrames += (uint32_t)(delta - 1U);
        }

        s_lastSequence = sequence;
    }

    s_latest.sequence      = sequence;
    s_latest.timestampMs   = AimProtocol_ReadLe32(&s_frame[6]);
    s_latest.rectX         = AimProtocol_ReadLe16(&s_frame[10]);
    s_latest.rectY         = AimProtocol_ReadLe16(&s_frame[12]);
    s_latest.laserX        = AimProtocol_ReadLe16(&s_frame[14]);
    s_latest.laserY        = AimProtocol_ReadLe16(&s_frame[16]);
    s_latest.validFlags    = s_frame[18];
    s_latest.trackingState = s_frame[19];
    s_latest.rxTimeMs      = Timer_GetMillis();

    s_hasLatest = 1U;
    s_stats.validFrames++;
}

static void AimProtocol_ResyncFromFailedFrame(void)
{
    uint8_t firstByte;
    uint8_t i;

    s_stats.headerResyncs++;

    firstByte = s_frame[1];
    if (firstByte == AIM_HEADER0)
    {
        s_frame[0] = AIM_HEADER0;
        s_frameIndex = 1U;
        s_parseState = AIM_PARSE_WAIT_HEADER1;
        return;
    }

    for (i = 1U; i < AIM_FRAME_SIZE; i++)
    {
        if (s_frame[i] == AIM_HEADER0)
        {
            uint8_t j;
            uint8_t remain = (uint8_t)(AIM_FRAME_SIZE - i);

            for (j = 0U; j < remain; j++)
            {
                s_frame[j] = s_frame[i + j];
            }

            if (remain >= 2U && s_frame[1] == AIM_HEADER1)
            {
                s_frameIndex = 2U;
                s_parseState = AIM_PARSE_COLLECT;
            }
            else
            {
                s_frameIndex = 1U;
                s_parseState = AIM_PARSE_WAIT_HEADER1;
            }
            return;
        }
    }

    s_parseState = AIM_PARSE_WAIT_HEADER0;
}

void AimProtocol_Init(void)
{
    uint8_t i;
    for (i = 0U; i < AIM_FRAME_SIZE; i++) { s_frame[i] = 0U; }
    s_frameIndex     = 0U;
    s_parseState     = AIM_PARSE_WAIT_HEADER0;
    s_hasLatest      = 0U;
    s_hasLastSequence = 0U;
    s_lastSequence   = 0U;

    for (i = 0U; i < (uint8_t)sizeof(AimObservation_t); i++)
    {
        ((volatile uint8_t *)&s_latest)[i] = 0U;
    }
    for (i = 0U; i < (uint8_t)sizeof(AimProtocolStats_t); i++)
    {
        ((volatile uint8_t *)&s_stats)[i] = 0U;
    }
}

void AimProtocol_Process(void)
{
    uint8_t byte;

    while (K230Uart_ReadByte(&byte))
    {
        s_stats.rxBytes++;

        switch (s_parseState)
        {
            case AIM_PARSE_WAIT_HEADER0:
                if (byte == AIM_HEADER0)
                {
                    s_frame[0] = byte;
                    s_frameIndex = 1U;
                    s_parseState = AIM_PARSE_WAIT_HEADER1;
                }
                break;

            case AIM_PARSE_WAIT_HEADER1:
                if (byte == AIM_HEADER1)
                {
                    s_frame[1] = byte;
                    s_frameIndex = 2U;
                    s_parseState = AIM_PARSE_COLLECT;
                }
                else if (byte != AIM_HEADER0)
                {
                    s_parseState = AIM_PARSE_WAIT_HEADER0;
                }
                break;

            case AIM_PARSE_COLLECT:
                s_frame[s_frameIndex] = byte;
                s_frameIndex++;

                if (s_frameIndex >= AIM_FRAME_SIZE)
                {
                    s_stats.frameCandidates++;
                    if (AimProtocol_ValidateFrame())
                    {
                        AimProtocol_AcceptFrame();
                        s_parseState = AIM_PARSE_WAIT_HEADER0;
                    }
                    else
                    {
                        AimProtocol_ResyncFromFailedFrame();
                    }
                }
                break;

            default:
                s_parseState = AIM_PARSE_WAIT_HEADER0;
                break;
        }
    }
}

uint8_t AimProtocol_GetLatest(AimObservation_t *outObservation)
{
    uint32_t primask;

    if (outObservation == 0) { return 0U; }
    if (!s_hasLatest) { return 0U; }

    primask = __get_PRIMASK();
    __disable_irq();
    *outObservation = s_latest;
    if (primask == 0U) { __enable_irq(); }
    return 1U;
}

void AimProtocol_GetStats(AimProtocolStats_t *outStats)
{
    uint32_t primask;

    if (outStats == 0) { return; }

    primask = __get_PRIMASK();
    __disable_irq();
    *outStats = s_stats;
    if (primask == 0U) { __enable_irq(); }
}

uint8_t AimProtocol_HasValidObservation(void)
{
    return s_hasLatest;
}
