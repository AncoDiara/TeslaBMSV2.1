#pragma once
#include <Arduino.h>
#include "config.h"
#include "Logger.h"

/*
  Communication with the Tesla BMB boards (bq76PL536 chips) over SERIALBMS.

  Changes compared to TeslaBMSV2:
  - flushRx(): before every new command, old/stale bytes are discarded from the
    receive buffer. Previously a stray byte could shift a reply, after which a
    module was "not found".
  - getReplyTimed(): waits until the expected number of bytes has arrived (with
    a timeout) instead of reading only what happens to be available after a fixed delay.
  - checkReadReply(): validates address, register, length AND CRC of a read reply.
*/

class BMSUtil {
public:

    static uint8_t genCRC(uint8_t *input, int lenInput)
    {
        uint8_t generator = 0x07;
        uint8_t crc = 0;

        for (int x = 0; x < lenInput; x++)
        {
            crc ^= input[x]; /* XOR-in the next input byte */

            for (int i = 0; i < 8; i++)
            {
                if ((crc & 0x80) != 0)
                {
                    crc = (uint8_t)((crc << 1) ^ generator);
                }
                else
                {
                    crc <<= 1;
                }
            }
        }

        return crc;
    }

    // Discard everything still sitting in the receive buffer.
    static void flushRx()
    {
        while (SERIALBMS.available()) SERIALBMS.read();
    }

    static void sendData(uint8_t *data, uint8_t dataLen, bool isWrite)
    {
        uint8_t orig = data[0];
        uint8_t addrByte = data[0];
        if (isWrite) addrByte |= 1;

        flushRx(); // NEW: never let old bytes contaminate the next reply

        SERIALBMS.write(addrByte);
        SERIALBMS.write(&data[1], dataLen - 1);  //assumes that there are at least 2 bytes sent every time. There should be, addr and cmd at the least.
        data[0] = addrByte;
        if (isWrite) SERIALBMS.write(genCRC(data, dataLen));

        if (Logger::isDebug())
        {
            SERIALCONSOLE.print("Sending: ");
            SERIALCONSOLE.print(addrByte, HEX);
            SERIALCONSOLE.print(" ");
            for (int x = 1; x < dataLen; x++) {
                SERIALCONSOLE.print(data[x], HEX);
                SERIALCONSOLE.print(" ");
            }
            if (isWrite) SERIALCONSOLE.print(genCRC(data, dataLen), HEX);
            SERIALCONSOLE.println();
        }

        data[0] = orig;
    }

    // Original behaviour: read whatever is available right now (max maxLen).
    static int getReply(uint8_t *data, int maxLen)
    {
        int numBytes = 0;
        if (Logger::isDebug()) SERIALCONSOLE.print("Reply: ");
        while (SERIALBMS.available() && numBytes < maxLen)
        {
            data[numBytes] = SERIALBMS.read();
            if (Logger::isDebug()) {
                SERIALCONSOLE.print(data[numBytes], HEX);
                SERIALCONSOLE.print(" ");
            }
            numBytes++;
        }
        if (maxLen == numBytes)
        {
            while (SERIALBMS.available()) SERIALBMS.read();
        }
        if (Logger::isDebug()) SERIALCONSOLE.println();
        return numBytes;
    }

    // NEW: wait until 'expected' bytes have arrived, or until timeoutMs has elapsed.
    static int getReplyTimed(uint8_t *data, int expected, uint16_t timeoutMs)
    {
        int numBytes = 0;
        uint32_t start = millis();
        while (numBytes < expected && (millis() - start) <= timeoutMs)
        {
            if (SERIALBMS.available())
            {
                data[numBytes++] = SERIALBMS.read();
            }
        }
        if (Logger::isDebug())
        {
            SERIALCONSOLE.print("Reply: ");
            for (int x = 0; x < numBytes; x++) { SERIALCONSOLE.print(data[x], HEX); SERIALCONSOLE.print(" "); }
            SERIALCONSOLE.println();
        }
        return numBytes;
    }

    // NEW: check a reply to a READ command: [addr<<1][reg][len][data...][crc]
    static bool checkReadReply(uint8_t *buff, int retLen, uint8_t addr, uint8_t reg, uint8_t len)
    {
        if (retLen != len + 4) return false;
        if (buff[0] != (uint8_t)(addr << 1)) return false;
        if (buff[1] != reg) return false;
        if (buff[2] != len) return false;
        return (genCRC(buff, retLen - 1) == buff[retLen - 1]);
    }

    //Uses above functions to send data then get the response. Will auto retry if response not
    //the expected return length. This helps to alleviate any comm issues.
    static int sendDataWithReply(uint8_t *data, uint8_t dataLen, bool isWrite, uint8_t *retData, int retLen)
    {
        int attempts = 1;
        int returnedLength = 0;
        while (attempts < 4)
        {
            sendData(data, dataLen, isWrite);
            // IMPORTANT: always wait the same minimum time as in the original (TeslaBMSV2).
            // The modules need that time, among other things to complete an ADC measurement
            // (REG_ADC_CONV) before the voltage registers are read. Without this wait
            // the module returns its OLD values (bug in the first version of TeslaBMS_Victron:
            // frozen voltages).
            delay(2 * ((retLen / 8) + 1));
            // then wait (with a timeout) for any remaining bytes of a long chain.
            returnedLength = getReplyTimed(retData, retLen, 10);
            if (returnedLength == retLen) return returnedLength;
            attempts++;
        }
        return returnedLength; //failed to get a proper response.
    }
};
