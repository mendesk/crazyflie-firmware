/*
 * esp8266.c - Deck driver for the ESP8266 WiFi module
 */
#define DEBUG_MODULE "ESP8266"

#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "stm32fxxx.h"
#include "config.h"
#include "console.h"
#include "debug.h"
#include "deck.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "log.h"
#include "param.h"
#include "system.h"
#include "uart2.h"
#include "estimator_kalman.h"

#define MAX_LINE_LENGTH 100

static bool isInit = false;

/* AT+CWLAPOPT values, AT+CWLAP output also follows this order
bit 0: determines whether <ecn> will be shown in the result of AT+CWLAP.
bit 1: determines whether <ssid> will be shown in the result of AT+CWLAP.
bit 2: determines whether <rssi> will be shown in the result of AT+CWLAP.
bit 3: determines whether <mac> will be shown in the result of AT+CWLAP.
bit 4: determines whether <ch> will be shown in the result of AT+CWLAP.
bit 5: determines whether <freq offset> will be shown in the result of AT+CWLAP.
bit 6: determines whether <freq calibration> will be shown in the result of AT+CWLAP.
bit 7: determines whether <pairwise_cipher> will be shown in the result of AT+CWLAP.
bit 8: determines whether <group_cipher> will be shown in the result of AT+CWLAP.
bit 9: determines whether <bgn> will be shown in the result of AT+CWLAP.
bit 10: determines whether <wps> will be shown in the result of AT+CWLAP
*/

// Implemented AT commands
static uint8_t at_test[] = "AT\r\n";
//static uint8_t at_echo_off[] = "ATE0\r\n";
static uint8_t at_cwmode[] = "AT+CWMODE_CUR=1\r\n";
static uint8_t at_cwlapopt[] = "AT+CWLAPOPT=0,30\r\n";

// Struct for holding the scan responses
struct wifiSignal {
    uint8_t ssid[33];  // 0: SSIDs are limited to 32 bytes + \0
    int8_t rssi;  // 1: RSSI value in dbm (negative)
    uint64_t mac;  // 2: 48 bit mac address
    uint8_t channel;  // 3: WiFi channel
};

char buffer[MAX_LINE_LENGTH];
point_t position;

// Parameters
uint8_t scanOnDemand = 1; // 0 for periodic scanning, 1 for on demand scanning
uint8_t scanInterval = 5;  // how long to wait between end of scan and beginning of next scan (only if scanOnDemand==0)
uint8_t scanNow = 0;  // set to 1 to do an immediate scan
uint8_t scanNowDelay = 0;  // how many seconds to wait before doing scan
uint8_t atScanType = 0;  // AT+CWLAP <scan_type>: 0=active, 1=passive
uint16_t atScanTimeMin = 0;  // AT+CWLAP <scan_time_min>
uint16_t atScanTimeMax = 120;  // AT+CWLAP <scan_time_max>

uint8_t parseLine(char *lineBuffer, struct wifiSignal *signal) {
    /*
     * Parses a line to a wifiSignal. Returns 1 if successful, 0 if there was a parse error
     */
    uint8_t bracketOpen = 0;
    uint8_t quoteOpen = 0;
    uint8_t parametersParsed = 0;
    uint8_t parameterIndex = 0;
    uint8_t parameterBuffer[18];  // We'll use this for rssi, mac and channel

    //DEBUG_PRINT("parseLine lineBuffer: %s\n", lineBuffer);

    for (uint8_t i = 0; lineBuffer[i] != '\0'; i++) {
        if (! bracketOpen) {
            // We're not at the meat (yet), skip this part of the output
            if (lineBuffer[i] == '(') {
                bracketOpen = 1;
            }
            continue;
        }

        if (quoteOpen) {
            // We're in a string (ssid or mac address)
            if (lineBuffer[i] == '"') {
                quoteOpen = 0;
            }
            else if (parametersParsed == 0) {
                // ssid
                (signal->ssid)[parameterIndex++] = lineBuffer[i];
            }
            else if (parametersParsed == 2) {
                // mac
                parameterBuffer[parameterIndex++] = lineBuffer[i];
            }
            else {
                // error
                consolePrintf("Parse error! Parameters parsed: %u\n", parametersParsed);
                consolePrintf("Line: %s", lineBuffer);
                return 0;
            }
        }
        else if (lineBuffer[i] == '"') {
            quoteOpen = 1;
        }
        else if (lineBuffer[i] == ',' || lineBuffer[i] == ')') {
            // Last parameter will end with ')' instead of ','
            if (lineBuffer[i] == ')')
                bracketOpen = 0;

            // Done with this parameter, process it
            parameterBuffer[parameterIndex] = '\0';

            unsigned char mac[6];

            switch (parametersParsed) {
                case 0:
                    (signal->ssid)[parameterIndex++] = '\0';
                case 1:
                    signal->rssi = (int8_t)atoi((char*)parameterBuffer);
                    break;
                case 2:
                    // DEBUG_PRINT("MAC: [%s]\n", (char*)parameterBuffer);
                    sscanf((char*)parameterBuffer,
                           "%x:%x:%x:%x:%x:%x",
                           &mac[0],&mac[1],&mac[2],&mac[3],&mac[4],&mac[5]);

                    signal->mac = 0;
                    for (uint8_t j = 0; j<6; j++) {
                        // DEBUG_PRINT("c: %x (%u)\n", mac[j], mac[j]);
                        signal->mac |= (uint64_t)mac[j] << ((5 - j) * 8);
                    }
                    break;
                case 3:
                    signal->channel = (uint8_t)atoi((char*)parameterBuffer);
                    break;
            }

            ++parametersParsed;
            parameterIndex = 0;
            parameterBuffer[0] = '\0';
        }
        else {
            parameterBuffer[parameterIndex++] = lineBuffer[i];
        }
    }
    // DEBUG_PRINT("Done! Parameters parsed: %u\n", parametersParsed);
    return parametersParsed;
}


void readLine(char *lineBuffer) {
    char ch;
    lineBuffer[0] = '\0';

    uint8_t i = 0;

    while (i < MAX_LINE_LENGTH) {
        uart2Getchar(&ch);
        // uart2GetDataWithTimout(&cint);
        // ch = (char)cint;

        if (ch == '\r') {
            // Ignore carriage returns
            continue;
        }
        else if (ch != '\n') {
            // Not a newline
            // consolePutchar(ch);
            lineBuffer[i++] = ch;
        }
        else {
            // Newline
            lineBuffer[i] = '\0';
            // consolePutchar(ch);
            return;
        }
    }
}

void readUntilOk(char *lineBuffer, uint8_t cwlap) {
    DEBUG_PRINT("-- START READING --\n");
    lineBuffer[0] = '\0';
    struct wifiSignal signal = {};

    uint8_t lineCounter = 0;
    while (strncmp(lineBuffer, "OK", 2) != 0) {
        readLine(lineBuffer);
        lineCounter++;

        if (cwlap) {
            uint8_t resultFlag = parseLine(lineBuffer, &signal);
            if (resultFlag) {
                // Found a wifi signal
                consolePrintf("AP: %s, %d, %llx, %u\n", signal.ssid, signal.rssi, signal.mac, signal.channel);
            }
            else {
                // DEBUG_PRINT("P: -\n");
                consolePrintf("P: %s\n", lineBuffer);
            }
        }
    }
    DEBUG_PRINT("Read %u lines\n", lineCounter);
    DEBUG_PRINT("-- STOP READING --\n");
}

void scanWifiAccessPoints(void* arg) {
    // Wait 5 seconds for an orderly startup
    vTaskDelay(M2T(5000));

    // Turn echo off
    // consolePrintf("%s", at_echo_off);
    // uart2SendData(sizeof(at_echo_off), at_echo_off);
    // vTaskDelay(500);
    // readUntilOk(buffer);

    // Set station mode
    consolePrintf("%s", at_cwmode);
    uart2SendData(sizeof(at_cwmode), at_cwmode);
    vTaskDelay(500);
    readUntilOk(buffer, 0);

    // Set CWLAP options
    consolePrintf("%s", at_cwlapopt);
    uart2SendData(sizeof(at_cwlapopt), at_cwlapopt);
    vTaskDelay(500);
    readUntilOk(buffer, 0);

    while(1) {
        if (scanOnDemand == 1 && scanNow == 0) {
            // No scan requested, let's wait 100ms before we check again
            vTaskDelay(M2T(100));
            continue;
        }
        else if (scanOnDemand == 1 && scanNow == 1) {
            // Delay if we need to
            if (scanNowDelay > 0) {
                consolePrintf("Waiting %d seconds before starting scan", scanNowDelay);
                vTaskDelay(M2T(scanNowDelay * 1000));
            }
        }

        // Scan!
        // Build scan instruction
        // AT+CWLAP[=<ssid>,<mac>,<channel>,<scan_type>,<scan_time_min>,<scan_time_max>]
        uint8_t at_cwlap[26];
        sprintf((char*)at_cwlap, "AT+CWLAP=,,,%u,%u,%u\r\n", atScanType, atScanTimeMin, atScanTimeMax);

        // Get our position from the Kalman estimator
        estimatorKalmanGetEstimatedPos(&position);

        // Scan access points
        consolePrintf("%s", at_cwlap);
        consolePrintf("POS: x=%f y=%f z=%f\n", (double)position.x, (double)position.y, (double)position.z);
        uart2SendData(sizeof(at_cwlap), at_cwlap);

        // Check echo
        vTaskDelay(M2T(1000)); // wait 1 sec for feedback
        readLine(buffer);
        if (strncmp(buffer, "AT+CWLAP", 8) != 0) {
            consolePrintf("Echo not received, skipping\n");
            continue;
        }

        // Read the response
        readUntilOk(buffer, 1);

        if (scanOnDemand == 0) {
            // Wait scanInterval seconds before doing another scan
            consolePrintf("Finished scanning, waiting %d seconds for next scan...\n", scanInterval);
            vTaskDelay(M2T(scanInterval * 1000));
        }
        else {
            // Reset parameter
            scanNow = 0;
            consolePrintf("Finished scanning, waiting for next scan request...\n");
        }
    }
}

static void esp8266Init(DeckInfo *info)
{
    if (isInit)
        return;

    // Set baud rate
    vTaskDelay(500);
    uart2Init(115200);
    vTaskDelay(500);

    // Send test command
    DEBUG_PRINT("%s", at_test);
    uart2SendData(sizeof(at_test), at_test);
    vTaskDelay(500);
    readUntilOk(buffer, 0);

    DEBUG_PRINT("ESP8266 Deck initialized\n");

    // Create task to measure SSID / MAC / RSSI
    xTaskCreate(scanWifiAccessPoints, "scan wifi access points",
                4 * configMINIMAL_STACK_SIZE, NULL, 3, NULL);

    isInit = true;
}

static bool esp8266Test()
{
    if (isInit) {
        DEBUG_PRINT("ESP8266 Deck test passed\n");
        return true;
    }
    return false;
}

static const DeckDriver esp8266_deck = {
    //.vid = 0xBC,
    //.pid = 0x12,
    .name = "esp8266",

    .init = esp8266Init,
    .test = esp8266Test,
};

DECK_DRIVER(esp8266_deck);

PARAM_GROUP_START(esp8266)
    PARAM_ADD(PARAM_UINT8, scanOnDemand, &scanOnDemand)
    PARAM_ADD(PARAM_UINT8, scanInterval, &scanInterval)
    PARAM_ADD(PARAM_UINT8, scanNow, &scanNow)
    PARAM_ADD(PARAM_UINT8, scanNowDelay, &scanNowDelay)
    PARAM_ADD(PARAM_UINT8, atScanType, &atScanType)
    PARAM_ADD(PARAM_UINT16, atScanTimeMin, &atScanTimeMin)
    PARAM_ADD(PARAM_UINT16, atScanTimeMax, &atScanTimeMax)
PARAM_GROUP_STOP(esp8266)
