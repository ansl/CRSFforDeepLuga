/**
 * @file CRSF.cpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief This decodes CRSF frames from a serial port.
 *
 * @copyright Copyright (c) 2024, Cassandra "ZZ Cat" Robinson. All rights reserved.
 *
 * @section License GNU Affero General Public License v3.0
 * This source file is a part of the CRSF for Arduino library.
 * CRSF for Arduino is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CRSF for Arduino is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with CRSF for Arduino.  If not, see <https://www.gnu.org/licenses/>.
 * 
 */

#include "CRSF.hpp"
#include "../CRC/CRC_test.hpp"
#include "Arduino.h"
#include <cstdlib>
//luengoa
using namespace crsfProtocol;
using namespace genericCrc;

namespace serialReceiverLayer
{
    CRSF::CRSF()
    {

        rcFrameReceived = false;
        frameCount = 0;
        timePerFrame = 0;

        crc8 = new GenericCRC();
    }

    CRSF::CRSF(const CRSF &crsf)
    {
        rcFrameReceived = crsf.rcFrameReceived;
        frameCount = crsf.frameCount;
        timePerFrame = crsf.timePerFrame;

        memcpy(rxFrame.raw, crsf.rxFrame.raw, CRSF_FRAME_SIZE_MAX);
        memcpy(rcChannelsFrame.raw, crsf.rcChannelsFrame.raw, CRSF_FRAME_SIZE_MAX);

        crc8 = new GenericCRC(*crsf.crc8);
    }

    CRSF &CRSF::operator=(const CRSF &crsf)
    {
        if (this != &crsf)
        {
            rcFrameReceived = crsf.rcFrameReceived;
            frameCount = crsf.frameCount;
            timePerFrame = crsf.timePerFrame;

            memcpy(rxFrame.raw, crsf.rxFrame.raw, CRSF_FRAME_SIZE_MAX);
            memcpy(rcChannelsFrame.raw, crsf.rcChannelsFrame.raw, CRSF_FRAME_SIZE_MAX);
            memcpy(txFrame.raw, crsf.txFrame.raw, CRSF_FRAME_SIZE_MAX);
            *crc8 = *crsf.crc8;
        }

        return *this;
    }

    CRSF::~CRSF()
    {
        delete crc8;
        crc8 = nullptr;
    }

    void CRSF::begin()
    {
        rcFrameReceived = false;
        frameCount = 0;
        timePerFrame = 0;

        memset(rxFrame.raw, 0, CRSF_FRAME_SIZE_MAX);
        memset(rcChannelsFrame.raw, 0, CRSF_FRAME_SIZE_MAX);
    }

    void CRSF::end()
    {
        memset(rcChannelsFrame.raw, 0, CRSF_FRAME_SIZE_MAX);
        memset(rxFrame.raw, 0, CRSF_FRAME_SIZE_MAX);

        timePerFrame = 0;
        frameCount = 0;
        rcFrameReceived = false;
    }

    void CRSF::setFrameTime(uint32_t baudRate, uint8_t packetCount)
    {
        /* Calculate the time per frame based on the baud rate and packet count. */
        timePerFrame = ((1000000 * packetCount) / (baudRate / (CRSF_FRAME_SIZE_MAX - 1)));
    }

    bool CRSF::receiveFrames(uint8_t rxByte)
    {
        static bool frameReading = 0;
        static uint8_t framePosition = 0;
        static uint32_t frameStartTime = 0;
        const uint32_t currentTime = micros();

        if (rxByte == CRSF_ADDRESS_FLIGHT_CONTROLLER && framePosition == 0 && frameReading == 0) //|| currentTime - frameStartTime > timePerFrame)
        {
            frameReading = 1;
            framePosition = 0;
            frameStartTime = currentTime;
            memset(rxFrame.raw, 0, CRSF_FRAME_SIZE_MAX);
            if (currentTime < frameStartTime)
            {
                frameStartTime = currentTime;
            }
        }
        if (frameReading)
        {
            const int fullFrameLength = framePosition < 3 ? 5 : min(rxFrame.frame.frameLength + CRSF_FRAME_LENGTH_ADDRESS + CRSF_FRAME_LENGTH_FRAMELENGTH, (int)CRSF_FRAME_SIZE_MAX);

            // Serial.printf("%2X ; %d; %d ; %d ;%d\n", rxByte, framePosition, fullFrameLength, currentTime - frameStartTime, timePerFrame);
            // if (rxByte == 0xc8)
            // {
            //     Serial.println("xxxxxxxxxxxxxxxxxxxxxxx");
            // }
            // if (rxByte == 0x28)
            // {
            //     Serial.println("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx");
            // }
            /* Assume the full frame length is 5 bytes until the frame length byte is received. */

            if (framePosition < fullFrameLength)
            {
                /* Store the received byte in the frame buffer. */
                // if(rxByte==0x78){
                //             Serial.printf("%X\n",rxByte);
                // }
                rxFrame.raw[framePosition] = rxByte;
                framePosition++;

                if (framePosition >= fullFrameLength)
                {
                    /* Frame is complete, calculate the CRC and check if it is valid. */
                    const uint8_t crc = calculateFrameCRC();
                    if (rxFrame.frame.type != CRSF_FRAMETYPE_RC_CHANNELS_PACKED && rxFrame.frame.type != CRSF_FRAMETYPE_LINK_STATISTICS)
                    {
                        for (int i = 0; i < fullFrameLength; i++)
                        {
                            Serial.printf("%X ", rxFrame.raw[i]);
                        }
                        Serial.println();
                    }

                    if (crc == rxFrame.raw[fullFrameLength - 1])
                    {
                        //  Serial.printf("Received FRAME %x \n",rxFrame.frame.type);
                        switch (rxFrame.frame.type)
                        {
                            case crsfProtocol::CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
                                if (rxFrame.frame.deviceAddress == CRSF_ADDRESS_FLIGHT_CONTROLLER)
                                {
                                    memcpy(&rcChannelsFrame, &rxFrame, CRSF_FRAME_SIZE_MAX);
                                    rcFrameReceived = true;
                                }
                                // Serial.printf("Received: CRSF_FRAMETYPE_RC_CHANNELS_PACKED \n");
                                break;

#if CRSF_LINK_STATISTICS_ENABLED > 0
                            case CRSF_FRAMETYPE_LINK_STATISTICS:
                                if ((rxFrame.frame.deviceAddress == CRSF_ADDRESS_FLIGHT_CONTROLLER) && (rxFrame.frame.frameLength == CRSF_FRAME_ORIGIN_DEST_SIZE + CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE))
                                {
                                    crsf_payload_link_statistics_t linkStatisticsPayload;
                                    memcpy(&linkStatisticsPayload, rxFrame.frame.payload, sizeof(crsf_payload_link_statistics_t));

                                    linkStatistics.rssi = (linkStatisticsPayload.active_antenna ? linkStatisticsPayload.uplink_rssi_2 : linkStatisticsPayload.uplink_rssi_1);
                                    linkStatistics.lqi = linkStatisticsPayload.uplink_link_quality;
                                    linkStatistics.snr = linkStatisticsPayload.uplink_snr;
                                    linkStatistics.tx_power = (linkStatisticsPayload.uplink_tx_power < 9) ? tx_power_table[linkStatisticsPayload.uplink_tx_power] : 0;
                                }
                                // Serial.printf("Received: CRSF_FRAMETYPE_LINK_STATISTICS \n");
                                break;
#endif
                                // case CRSF_FRAMETYPE_COMMAND: //luengoa
                                //     Serial.printf("Received FRAME %X \n", rxFrame.frame.type);
                                //     Serial.printf("Received CMD %X \n", rxFrame.frame.payload[2]);
                                //     Serial.printf("Received subCMD %X \n", rxFrame.frame.payload[3]);
                                //     if (rxFrame.frame.payload[2] == 0x0A && rxFrame.frame.payload[3] == 0x70)
                                //     {                                                     //luengoa
                                //         Serial.println("BR NEgotiation");                 //luengoa
                                //         rx_answer = 1;                                    //luengoa
                                //         bdrate = 0x00000000;                              //luengoa
                                //         bdrate = bdrate | rxFrame.frame.payload[5] << 24; //luengoa
                                //         bdrate = bdrate | rxFrame.frame.payload[6] << 16; //luengoa
                                //         bdrate = bdrate | rxFrame.frame.payload[7] << 8;  //luengoa
                                //         bdrate = bdrate | rxFrame.frame.payload[8] << 0;  //luengoa
                                //         Serial.printf("Proposed BR: %d \n", bdrate);
                                //         if (bdrate > 416666)
                                //         // if (bdrate > 1000000)
                                //         {

                                //             uint8_t bff[] = {0xC8, 0x09, 0x32, 0xEC, 0xC8, 0x0A, 0x71, 0x00, 0x00, 0x00, 0x00}; //luengoa
                                //             bff[9] = crc8_ba(&bff[2], 7);                                                       //luengoa
                                //             bff[10] = crc8_5D(&bff[2], 8);                                                      //luengoa
                                //             memcpy(txFrame.raw, bff, 11);                                                       //luengoa

                                //             // Serial1->write(bff,12);
                                //             Serial.println("BR proposal rejected"); //luengoa
                                //             for (int i = 0; i < 11; i++)
                                //             {
                                //                 Serial.printf("%2X ", txFrame.raw[i]);
                                //             }
                                //             Serial.println();
                                //         }
                                //         else
                                //         {
                                //             uint8_t bff[] = {0xC8, 0x09, 0x32, 0xEC, 0xC8, 0x0A, 0x71, 0x00, 0x01, 0x00, 0x00}; //luengoa
                                //             bff[9] = crc8_ba(&bff[2], 7);                                                       //luengoa
                                //             bff[10] = crc8_5D(&bff[2], 8);                                                      //luengoa
                                //             memcpy(txFrame.raw, bff, 11);                                                       //luengoa
                                //             // Serial1->write(bff,12);
                                //             Serial.println("BR proposal Approved"); //luengoa
                                //             for (int i = 0; i < 11; i++)
                                //             {
                                //                 Serial.printf("%2X ", txFrame.raw[i]);
                                //             }
                                //             Serial.println();
                                //             BR_change = 1;
                                //         }
                                //     }
                                //     else if (rxFrame.frame.payload[2] == 0xFF && rxFrame.frame.payload[3] == 0x0A)
                                //     {
                                //         rx_answer = 1;
                                //         uint8_t bff[] = {0xC8, 0x0A, 0x32, 0xEC, 0xC8, 0xFF, 0x0A, 0x71, 0x01, 0x00, 0x00, 0x00}; //luengoa
                                //         bff[10] = crc8_ba(&bff[2], 8);                                                            //luengoa
                                //         bff[11] = crc8_5D(&bff[2], 9);                                                            //luengoa
                                //         memcpy(txFrame.raw, bff, 12);                                                             //luengoa
                                //         // Serial1->write(bff,12);
                                //         Serial.println("BR change ACK"); //luengoa
                                //         for (int i = 0; i < 11; i++)
                                //         {
                                //             Serial.printf("%2X ", txFrame.raw[i]);
                                //         }
                                //         Serial.println();
                                //     }
                                //     break;

                            case CRSF_FRAMETYPE_DEVICE_PING:
                                {
                                    Serial.printf("Received FRAME %X received \n", rxFrame.frame.type);
                                    rx_answer = 1;
                                    uint8_t bff[29] = {0xC8, 0x00, 0x29, 0xEA, 0xC8, 0x44, 0x65, 0x65, 0x70, 0x4C, 0x75, 0x67, 0x61, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00}; //luengoa
                                    // bff[10] = crc8_ba(&bff[2], 8);   
                                    bff[1]=sizeof(bff)-2;                                                         //luengoa
                                    bff[sizeof(bff) - 1] = crc8_5D(&bff[2], sizeof(bff) - 3); //luengoa
                                    memcpy(txFrame.raw, bff, sizeof(bff));
                                }
                                break;
                            case CRSF_FRAMETYPE_BARBUS_SEND_PERI:
                                {
                                    Serial.printf("Received FRAME %X received \n", rxFrame.frame.type);
                                    
                                    double coord[5][2];
                                    memset (coord,0,10*sizeof(double)); 
                                    memcpy(coord,&rxFrame.raw[7],10*sizeof(double));
                                    // coord=*reinterpret_cast<const double*>(&rxFrame.raw[5])
                                    rx_answer = 1;
                                    uint8_t bff[29] = {0xC8, 0x00, CRSF_FRAMETYPE_BARBUS_ACK, CRSF_ADDRESS_RADIO_TRANSMITTER,CRSF_ADDRESS_FLIGHT_CONTROLLER,CRSF_FRAMETYPE_BARBUS_SEND_PERI, rxFrame.raw[6], 0x00}; //luengoa -- [sync] [len] [type] [DEST] [ORIG] [COMMAND to ACK] [CHUNK_N][payload] [crc8]
                                    bff[1]=sizeof(bff)-2; //luengoa
                                    bff[sizeof(bff) - 1] = crc8_5D(&bff[2], sizeof(bff) - 3); //luengoa
                                    memcpy(txFrame.raw, bff, sizeof(bff));

                                    Serial.printf("Received coords  lat=%1.7f  lon=%1.7f  lat=%1.7f  lon=%1.7f  \n", coord[0][0],coord[0][1], coord[1][0],coord[1][1]);


                                }
                                //luengoa

                                break;
                            default:
                                Serial.printf("Received FRAME %X received\n", rxFrame.frame.type);
                        }
                    }

                    /* Clear the frame buffer and reset the frame position. */
                    memset(rxFrame.raw, 0, CRSF_FRAME_SIZE_MAX);
                    framePosition = 0;
                    frameReading = 0;
                    return true;
                }
            }
        }
        return false;
    }

    void CRSF::getFailSafe(bool *failSafe)
    {
        /* Set the failsafe flag based on the link statistics thresholds. */
        if (linkStatistics.lqi <= CRSF_FAILSAFE_LQI_THRESHOLD || linkStatistics.rssi >= CRSF_FAILSAFE_RSSI_THRESHOLD)
        {
            *failSafe = true;
        }
        else
        {
            *failSafe = false;
        }
    }

    void CRSF::getRcChannels(uint16_t *rcChannels)
    {
        /* Decode RC frames if one has been received. */
        if (rcFrameReceived)
        {
            rcFrameReceived = false;
            if (rcChannelsFrame.frame.type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
            {
                rcChannelsPacked_t rcChannelsPacked;
                memcpy(&rcChannelsPacked, rcChannelsFrame.frame.payload, sizeof(rcChannelsPacked_t));

                rcChannels[RC_CHANNEL_ROLL] = rcChannelsPacked.channel0;
                rcChannels[RC_CHANNEL_PITCH] = rcChannelsPacked.channel1;
                rcChannels[RC_CHANNEL_THROTTLE] = rcChannelsPacked.channel2;
                rcChannels[RC_CHANNEL_YAW] = rcChannelsPacked.channel3;
                rcChannels[RC_CHANNEL_AUX1] = rcChannelsPacked.channel4;
                rcChannels[RC_CHANNEL_AUX2] = rcChannelsPacked.channel5;
                rcChannels[RC_CHANNEL_AUX3] = rcChannelsPacked.channel6;
                rcChannels[RC_CHANNEL_AUX4] = rcChannelsPacked.channel7;
                rcChannels[RC_CHANNEL_AUX5] = rcChannelsPacked.channel8;
                rcChannels[RC_CHANNEL_AUX6] = rcChannelsPacked.channel9;
                rcChannels[RC_CHANNEL_AUX7] = rcChannelsPacked.channel10;
                rcChannels[RC_CHANNEL_AUX8] = rcChannelsPacked.channel11;
                rcChannels[RC_CHANNEL_AUX9] = rcChannelsPacked.channel12;
                rcChannels[RC_CHANNEL_AUX10] = rcChannelsPacked.channel13;
                rcChannels[RC_CHANNEL_AUX11] = rcChannelsPacked.channel14;
                rcChannels[RC_CHANNEL_AUX12] = rcChannelsPacked.channel15;
            }
        }
    }

    void CRSF::getLinkStatistics(link_statistics_t *linkStats)
    {
#if CRSF_LINK_STATISTICS_ENABLED > 0
        memcpy(linkStats, &linkStatistics, sizeof(link_statistics_t));
#else
        (void)linkStats;
#endif
    }

    uint8_t CRSF::calculateFrameCRC()
    {
        return crc8->calculate(rxFrame.frame.type, rxFrame.frame.payload, rxFrame.frame.frameLength - CRSF_FRAME_LENGTH_TYPE_CRC);
    }
} // namespace serialReceiverLayer
