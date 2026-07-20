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
static uint32_t s_lastAcceptedRxTimeMs;

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

uint8_t AimProtocol_RunSelfTest(void)
{
    uint16_t crc;
    const uint8_t testStr[] = "123456789";
    uint16_t le16test;
    uint32_t le32test;
    uint8_t le16raw[2] = {0x34, 0x12};
    uint8_t le32raw[4] = {0x78, 0x56, 0x34, 0x12};

    crc = AimProtocol_Crc16CcittFalse(testStr, 9U);
    if (crc != 0x29B1U) { return 0U; }

    le16test = AimProtocol_ReadLe16(le16raw);
    if (le16test != 0x1234U) { return 0U; }

    le32test = AimProtocol_ReadLe32(le32raw);
    if (le32test != 0x12345678UL) { return 0U; }

    return 1U;
}

static uint8_t AimProtocol_IsXInRange(uint16_t value)
{
    return (uint8_t)((value < AIM_IMAGE_WIDTH) ? 1U : 0U);
}

static uint8_t AimProtocol_IsYInRange(uint16_t value)
{
    return (uint8_t)((value < AIM_IMAGE_HEIGHT) ? 1U : 0U);
}

static uint8_t AimProtocol_IsCoordValidOrInvalidX(uint16_t value)
{
    return (uint8_t)((value < AIM_IMAGE_WIDTH || value == AIM_COORD_INVALID) ? 1U : 0U);
}

static uint8_t AimProtocol_IsCoordValidOrInvalidY(uint16_t value)
{
    return (uint8_t)((value < AIM_IMAGE_HEIGHT || value == AIM_COORD_INVALID) ? 1U : 0U);
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

    if (!AimProtocol_IsCoordValidOrInvalidX(rectX) ||
        !AimProtocol_IsCoordValidOrInvalidY(rectY) ||
        !AimProtocol_IsCoordValidOrInvalidX(laserX) ||
        !AimProtocol_IsCoordValidOrInvalidY(laserY))
    {
        s_stats.fieldErrors++;
        return 0U;
    }

    if ((validFlags & AIM_FLAG_RECT_VALID) != 0U)
    {
        if (!AimProtocol_IsXInRange(rectX) || !AimProtocol_IsYInRange(rectY))
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
        if (!AimProtocol_IsXInRange(laserX) || !AimProtocol_IsYInRange(laserY))
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
    uint32_t nowMs;

    sequence = AimProtocol_ReadLe16(&s_frame[4]);
    nowMs = Timer_GetMillis();

    if (!s_hasLastSequence)
    {
        s_hasLastSequence          = 1U;
        s_lastSequence             = sequence;
        s_lastAcceptedRxTimeMs     = nowMs;
    }
    else
    {
        uint32_t elapsed = nowMs - s_lastAcceptedRxTimeMs;

        if (elapsed > AIM_SEQUENCE_REBASE_TIMEOUT_MS)
        {
            s_stats.sequenceRebases++;
            s_lastSequence         = sequence;
            s_lastAcceptedRxTimeMs = nowMs;
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

            s_lastSequence         = sequence;
            s_lastAcceptedRxTimeMs = nowMs;
        }
    }

    s_latest.sequence      = sequence;
    s_latest.timestampMs   = AimProtocol_ReadLe32(&s_frame[6]);
    s_latest.rectX         = AimProtocol_ReadLe16(&s_frame[10]);
    s_latest.rectY         = AimProtocol_ReadLe16(&s_frame[12]);
    s_latest.laserX        = AimProtocol_ReadLe16(&s_frame[14]);
    s_latest.laserY        = AimProtocol_ReadLe16(&s_frame[16]);
    s_latest.validFlags    = s_frame[18];
    s_latest.trackingState = s_frame[19];
    s_latest.rxTimeMs      = nowMs;

    s_hasLatest = 1U;
    s_stats.validFrames++;
}

static void AimProtocol_ResyncFromFailedFrame(void)
{
    uint8_t i;

    s_stats.headerResyncs++;

    /*
     * Start from index 1 (NOT 0) because index 0 is always the failed
     * frame's own AA 55 header.  Matching i=0 would give remain=22
     * and cause the next byte to write s_frame[22] (out of bounds).
     */
    for (i = 1U; i < (AIM_FRAME_SIZE - 1U); i++)
    {
        if (s_frame[i] == AIM_HEADER0 && s_frame[i + 1U] == AIM_HEADER1)
        {
            uint8_t j;
            uint8_t remain = (uint8_t)(AIM_FRAME_SIZE - i);

            for (j = 0U; j < remain; j++)
            {
                s_frame[j] = s_frame[i + j];
            }

            s_frameIndex = remain;
            s_parseState = AIM_PARSE_COLLECT;
            return;
        }
    }

    /*
     * No complete AA 55 found in the payload. Check if the last byte
     * is a lone 0xAA that could be the start of the next frame.
     */
    if (s_frame[AIM_FRAME_SIZE - 1U] == AIM_HEADER0)
    {
        s_frame[0]   = AIM_HEADER0;
        s_frameIndex = 1U;
        s_parseState = AIM_PARSE_WAIT_HEADER1;
    }
    else
    {
        s_frameIndex = 0U;
        s_parseState = AIM_PARSE_WAIT_HEADER0;
    }
}

void AimProtocol_Init(void)
{
    uint16_t i;

    for (i = 0U; i < AIM_FRAME_SIZE; i++) { s_frame[i] = 0U; }
    s_frameIndex            = 0U;
    s_parseState            = AIM_PARSE_WAIT_HEADER0;
    s_hasLatest             = 0U;
    s_hasLastSequence       = 0U;
    s_lastSequence          = 0U;
    s_lastAcceptedRxTimeMs  = 0U;

    for (i = 0U; i < (uint16_t)sizeof(s_latest); i++)
    {
        ((uint8_t *)&s_latest)[i] = 0U;
    }
    for (i = 0U; i < (uint16_t)sizeof(s_stats); i++)
    {
        ((uint8_t *)&s_stats)[i] = 0U;
    }

    s_stats.selfTestPassed = AimProtocol_RunSelfTest();
}

void AimProtocol_Process(void)
{
    uint8_t byte;

    if (s_stats.selfTestPassed == 0U) { return; }

    while (K230Uart_ReadByte(&byte))
    {
        s_stats.rxBytes++;

        switch (s_parseState)
        {
            case AIM_PARSE_WAIT_HEADER0:
                if (byte == AIM_HEADER0)
                {
                    s_frame[0]   = byte;
                    s_frameIndex = 1U;
                    s_parseState = AIM_PARSE_WAIT_HEADER1;
                }
                break;

            case AIM_PARSE_WAIT_HEADER1:
                if (byte == AIM_HEADER1)
                {
                    s_frame[1]   = byte;
                    s_frameIndex = 2U;
                    s_parseState = AIM_PARSE_COLLECT;
                }
                else if (byte != AIM_HEADER0)
                {
                    s_frameIndex = 0U;
                    s_parseState = AIM_PARSE_WAIT_HEADER0;
                }
                break;

            case AIM_PARSE_COLLECT:
                /*
                 * Guard: NEVER write beyond s_frame[21].
                 * If index is already out of bounds, discard and restart.
                 */
                if (s_frameIndex >= AIM_FRAME_SIZE)
                {
                    s_stats.parserGuardResets++;
                    s_frameIndex = 0U;
                    s_parseState = AIM_PARSE_WAIT_HEADER0;
                    break;
                }

                s_frame[s_frameIndex] = byte;
                s_frameIndex++;

                if (s_frameIndex >= AIM_FRAME_SIZE)
                {
                    s_stats.frameCandidates++;
                    if (AimProtocol_ValidateFrame())
                    {
                        AimProtocol_AcceptFrame();
                    }
                    else
                    {
                        AimProtocol_ResyncFromFailedFrame();
                    }
                    /*
                     * After processing a complete frame (valid or failed),
                     * state and frameIndex are set by AcceptFrame() or
                     * ResyncFromFailedFrame().  If neither sets the state
                     * (AcceptFrame does not reset), we MUST reset here.
                     */
                    if (s_parseState == AIM_PARSE_COLLECT)
                    {
                        s_frameIndex = 0U;
                        s_parseState = AIM_PARSE_WAIT_HEADER0;
                    }
                }
                break;

            default:
                s_frameIndex = 0U;
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
