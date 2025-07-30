/*--------------------------------------------------------------------
This file is part of the Arduino WiFiEsp library.

The Arduino WiFiEsp library is free software: you can redistribute it
and/or modify it under the terms of the GNU General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.

The Arduino WiFiEsp library is distributed in the hope that it will be
useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with The Arduino WiFiEsp library.  If not, see
<http://www.gnu.org/licenses/>.
--------------------------------------------------------------------*/

#include <Arduino.h>
#include <avr/pgmspace.h>

#include "EspDrv.h"
#include "debug.h"

#define vsnprintf_P vsnprintf

#define NUMESPTAGS 5
#define IN_PROGRESS -100

const char* ESPTAGS[] =
{
    "\r\nOK\r\n",
    "\r\nERROR\r\n",
    "\r\nFAIL\r\n",
    "\r\nSEND OK\r\n",
    " CONNECT\r\n"
};

typedef enum
{
    TAG_OK,
    TAG_ERROR,
    TAG_FAIL,
    TAG_SENDOK,
    TAG_CONNECT
} TagsEnum;


Stream *EspDrv::espSerial;

ERingBuffer EspDrv::ringBuf(32);

// Array of data to cache the information related to the networks discovered
char     EspDrv::_networkSsid[][WL_SSID_MAX_LENGTH] = {{"1"},{"2"},{"3"},{"4"},{"5"}};
int32_t EspDrv::_networkRssi[WL_NETWORKS_LIST_MAXNUM] = { 0 };
uint8_t EspDrv::_networkEncr[WL_NETWORKS_LIST_MAXNUM] = { 0 };

// Cached values of retrieved data
char EspDrv::_ssid[] = {0};
uint8_t EspDrv::_bssid[] = {0};
uint8_t EspDrv::_mac[] = {0};
uint8_t EspDrv::_localIp[] = {0};
char EspDrv::fwVersion[] = {0};

long EspDrv::_bufPos=0;
uint8_t EspDrv::_connId=0;

uint16_t EspDrv::_remotePort  =0;
uint8_t EspDrv::_remoteIp[] = {0};


void EspDrv::wifiDriverInit(Stream *espSerial)
{
    LOGDEBUG(F("> wifiDriverInit"));

    EspDrv::espSerial = espSerial;

    bool initOK = false;
    
    for(int i=0; i<5; i++)
    {
        if (sendCmd(F("AT")) == TAG_OK)
        {
            initOK=true;
            break;
        }
        delay(1000);
    }

    if (!initOK)
    {
        LOGERROR(F("Cannot initialize ESP module"));
        delay(5000);
        return;
    }

    reset();

    // check firmware version
    getFwVersion();

    // prints a warning message if the firmware is not 1.X or 2.X
    if ((fwVersion[0] != '1' and fwVersion[0] != '2') or
        fwVersion[1] != '.')
    {
        LOGWARN1(F("Warning: Unsupported firmware"), fwVersion);
        delay(4000);
    }
    else
    {
        LOGINFO1(F("Initilization successful -"), fwVersion);
    }
}


void EspDrv::reset()
{
    static enum {
        RST_IDLE, RST_SEND_RST, RST_WAIT_RST, RST_EMPTY_BUF,
        RST_SEND_ATE0, RST_SEND_CWMODE, RST_WAIT_CWMODE,
        RST_SEND_CIPMUX, RST_SEND_CIPDINFO, RST_SEND_CWAUTOCONN,
        RST_SEND_CWDHCP, RST_DONE
    } state = RST_IDLE;
    static unsigned long startTime = 0;

    switch (state) {
        case RST_IDLE:
            LOGDEBUG(F("> reset"));
            sendCmd(F("AT+RST"));
            startTime = millis();
            state = RST_WAIT_RST;
            return;

        case RST_WAIT_RST:
            if (millis() - startTime > 3000) {
                state = RST_EMPTY_BUF;
            }
            return;

        case RST_EMPTY_BUF:
            espEmptyBuf(false);  // empty dirty characters from the buffer
            state = RST_SEND_ATE0;
            return;

        case RST_SEND_ATE0:
            sendCmd(F("ATE0"));
            state = RST_SEND_CWMODE;
            return;

        case RST_SEND_CWMODE:
            sendCmd(F("AT+CWMODE=1"));
            startTime = millis();
            state = RST_WAIT_CWMODE;
            return;

        case RST_WAIT_CWMODE:
            if (millis() - startTime > 200) {
                state = RST_SEND_CIPMUX;
            }
            return;

        case RST_SEND_CIPMUX:
            sendCmd(F("AT+CIPMUX=1"));
            state = RST_SEND_CIPDINFO;
            return;

        case RST_SEND_CIPDINFO:
            sendCmd(F("AT+CIPDINFO=1"));
            state = RST_SEND_CWAUTOCONN;
            return;

        case RST_SEND_CWAUTOCONN:
            sendCmd(F("AT+CWAUTOCONN=0"));
            state = RST_SEND_CWDHCP;
            return;

        case RST_SEND_CWDHCP:
            sendCmd(F("AT+CWDHCP=1,1"));
            startTime = millis();
            state = RST_DONE;
            return;

        case RST_DONE:
            if (millis() - startTime > 200) {
                state = RST_IDLE;
            }
            return;
    }
}


bool EspDrv::wifiConnect(const char* ssid, const char* passphrase)
{
    // Static state variables to persist between calls
    static enum { WIFI_IDLE, WIFI_SEND_CMD, WIFI_WAIT_RESP, WIFI_DONE, WIFI_FAIL } state = WIFI_IDLE;
    static unsigned long startTime = 0;
    static char lastSsid[WL_SSID_MAX_LENGTH] = {0};
    static char lastPass[WL_SSID_MAX_LENGTH] = {0};
    static ERingBuffer respBuf(64);

    switch (state) {
        case WIFI_IDLE:
            strncpy(lastSsid, ssid, WL_SSID_MAX_LENGTH-1);
            strncpy(lastPass, passphrase, WL_SSID_MAX_LENGTH-1);
            lastSsid[WL_SSID_MAX_LENGTH-1] = 0;
            lastPass[WL_SSID_MAX_LENGTH-1] = 0;
            espSerial->flush();
            espEmptyBuf(false);
            espSerial->print(F("AT+CWJAP_CUR=\""));
            espSerial->print(lastSsid);
            espSerial->print(F("\",\""));
            espSerial->print(lastPass);
            espSerial->println(F("\""));
            startTime = millis();
            respBuf.init();
            state = WIFI_WAIT_RESP;
            return false;

        case WIFI_WAIT_RESP:
            while (espSerial->available()) {
                char c = espSerial->read();
                respBuf.push(c);
                if (respBuf.endsWith("\r\nOK\r\n")) {
                    LOGINFO1(F("Connected to"), lastSsid);
                    state = WIFI_DONE;
                    break;
                }
                if (respBuf.endsWith("\r\nFAIL\r\n") || respBuf.endsWith("\r\nERROR\r\n")) {
                    LOGWARN1(F("Failed connecting to"), lastSsid);
                    state = WIFI_FAIL;
                    break;
                }
            }
            if (state == WIFI_DONE) {
                state = WIFI_IDLE;
                return true;
            }
            if (state == WIFI_FAIL) {
                espEmptyBuf(false);
                state = WIFI_IDLE;
                return false;
            }
            // Timeout after 20 seconds
            if (millis() - startTime > 20000) {
                LOGWARN1(F("Timeout connecting to"), lastSsid);
                espEmptyBuf(false);
                state = WIFI_IDLE;
                return false;
            }
            return false;

        default:
            state = WIFI_IDLE;
            return false;
    }
}


bool EspDrv::wifiStartAP(const char* ssid, const char* pwd, uint8_t channel, uint8_t enc, uint8_t espMode)
{
	LOGDEBUG(F("> wifiStartAP"));

	// set AP mode, use CUR mode to avoid automatic start at boot
    int ret = sendCmd(F("AT+CWMODE_CUR=%d"), 10000, espMode);
	if (ret!=TAG_OK)
	{
		LOGWARN1(F("Failed to set AP mode"), ssid);
		return false;
	}

	// TODO
	// Escape character syntax is needed if "SSID" or "password" contains
	// any special characters (',', '"' and '/')

	// start access point
	ret = sendCmd(F("AT+CWSAP_CUR=\"%s\",\"%s\",%d,%d"), 10000, ssid, pwd, channel, enc);

	if (ret!=TAG_OK)
	{
		LOGWARN1(F("Failed to start AP"), ssid);
		return false;
	}
	
	if (espMode==2)
		sendCmd(F("AT+CWDHCP_CUR=0,1"));    // enable DHCP for AP mode
	if (espMode==3)
		sendCmd(F("AT+CWDHCP_CUR=2,1"));    // enable DHCP for station and AP mode

	LOGINFO1(F("Access point started"), ssid);
	return true;
}


int8_t EspDrv::disconnect()
{
	LOGDEBUG(F("> disconnect"));

	if(sendCmd(F("AT+CWQAP"))==TAG_OK)
		return WL_DISCONNECTED;

	// wait and clear any additional message
	delay(2000);
	espEmptyBuf(false);

	return WL_DISCONNECTED;
}

void EspDrv::config(IPAddress ip)
{
	LOGDEBUG(F("> config"));

	// disable station DHCP
	sendCmd(F("AT+CWDHCP_CUR=1,0"));
	
	// it seems we need to wait here...
	delay(500);
	
	char buf[16];
	sprintf_P(buf, PSTR("%d.%d.%d.%d"), ip[0], ip[1], ip[2], ip[3]);

	int ret = sendCmd(F("AT+CIPSTA_CUR=\"%s\""), 2000, buf);
	delay(500);

	if (ret==TAG_OK)
	{
		LOGINFO1(F("IP address set"), buf);
	}
}

void EspDrv::configAP(IPAddress ip)
{
	LOGDEBUG(F("> config"));
	
    sendCmd(F("AT+CWMODE_CUR=2"));
	
	// disable station DHCP
	sendCmd(F("AT+CWDHCP_CUR=2,0"));
	
	// it seems we need to wait here...
	delay(500);
	
	char buf[16];
	sprintf_P(buf, PSTR("%d.%d.%d.%d"), ip[0], ip[1], ip[2], ip[3]);

	int ret = sendCmd(F("AT+CIPAP_CUR=\"%s\""), 2000, buf);
	delay(500);

	if (ret==TAG_OK)
	{
		LOGINFO1(F("IP address set"), buf);
	}
}

uint8_t EspDrv::getConnectionStatus()
{
	LOGDEBUG(F("> getConnectionStatus"));

/*
	AT+CIPSTATUS

	Response

		STATUS:<stat>
		+CIPSTATUS:<link ID>,<type>,<remote_IP>,<remote_port>,<local_port>,<tetype>

	Parameters

		<stat>
			2: Got IP
			3: Connected
			4: Disconnected
		<link ID> ID of the connection (0~4), for multi-connect
		<type> string, "TCP" or "UDP"
		<remote_IP> string, remote IP address.
		<remote_port> remote port number
		<local_port> ESP8266 local port number
		<tetype>
			0: ESP8266 runs as client
			1: ESP8266 runs as server
*/

	char buf[10];
	if(!sendCmdGet(F("AT+CIPSTATUS"), F("STATUS:"), F("\r\n"), buf, sizeof(buf)))
		return WL_NO_SHIELD;

	// 4: client disconnected
	// 5: wifi disconnected
	int s = atoi(buf);
	if(s==2 or s==3 or s==4)
		return WL_CONNECTED;
	else if(s==5)
		return WL_DISCONNECTED;

	return WL_IDLE_STATUS;
}

uint8_t EspDrv::getClientState(uint8_t sock)
{
	LOGDEBUG1(F("> getClientState"), sock);

	char findBuf[20];
	sprintf_P(findBuf, PSTR("+CIPSTATUS:%d,"), sock);

	char buf[10];
	if (sendCmdGet(F("AT+CIPSTATUS"), findBuf, ",", buf, sizeof(buf)))
	{
		LOGDEBUG(F("Connected"));
		return true;
	}

	LOGDEBUG(F("Not connected"));
	return false;
}

uint8_t* EspDrv::getMacAddress()
{
	LOGDEBUG(F("> getMacAddress"));

	memset(_mac, 0, WL_MAC_ADDR_LENGTH);

	char buf[20];
	if (sendCmdGet(F("AT+CIFSR"), F(":STAMAC,\""), F("\""), buf, sizeof(buf)))
	{
		char* token;

		token = strtok(buf, ":");
		_mac[5] = (byte)strtol(token, NULL, 16);
		token = strtok(NULL, ":");
		_mac[4] = (byte)strtol(token, NULL, 16);
		token = strtok(NULL, ":");
		_mac[3] = (byte)strtol(token, NULL, 16);
		token = strtok(NULL, ":");
		_mac[2] = (byte)strtol(token, NULL, 16);
		token = strtok(NULL, ":");
		_mac[1] = (byte)strtol(token, NULL, 16);
		token = strtok(NULL, ":");
		_mac[0] = (byte)strtol(token, NULL, 16);
	}
	return _mac;
}


void EspDrv::getIpAddress(IPAddress& ip)
{
	LOGDEBUG(F("> getIpAddress"));

	char buf[20];
	memset(buf, '\0', sizeof(buf));
	if (sendCmdGet(F("AT+CIFSR"), F(":STAIP,\""), F("\""), buf, sizeof(buf)))
	{
		char* token;

		token = strtok(buf, ".");
		_localIp[0] = atoi(token);
		token = strtok(NULL, ".");
		_localIp[1] = atoi(token);
		token = strtok(NULL, ".");
		_localIp[2] = atoi(token);
		token = strtok(NULL, ".");
		_localIp[3] = atoi(token);

		ip = _localIp;
	}
}

void EspDrv::getIpAddressAP(IPAddress& ip)
{
	LOGDEBUG(F("> getIpAddressAP"));

	char buf[20];
	memset(buf, '\0', sizeof(buf));
	if (sendCmdGet(F("AT+CIPAP?"), F("+CIPAP:ip:\""), F("\""), buf, sizeof(buf)))
	{
		char* token;

		token = strtok(buf, ".");
		_localIp[0] = atoi(token);
		token = strtok(NULL, ".");
		_localIp[1] = atoi(token);
		token = strtok(NULL, ".");
		_localIp[2] = atoi(token);
		token = strtok(NULL, ".");
		_localIp[3] = atoi(token);

		ip = _localIp;
	}
}



char* EspDrv::getCurrentSSID()
{
	LOGDEBUG(F("> getCurrentSSID"));

	_ssid[0] = 0;
	sendCmdGet(F("AT+CWJAP?"), F("+CWJAP:\""), F("\""), _ssid, sizeof(_ssid));

	return _ssid;
}

uint8_t* EspDrv::getCurrentBSSID()
{
	LOGDEBUG(F("> getCurrentBSSID"));

	memset(_bssid, 0, WL_MAC_ADDR_LENGTH);

	char buf[20];
	if (sendCmdGet(F("AT+CWJAP?"), F(",\""), F("\","), buf, sizeof(buf)))
	{
		char* token;

		token = strtok(buf, ":");
		_bssid[5] = (byte)strtol(token, NULL, 16);
		token = strtok(NULL, ":");
		_bssid[4] = (byte)strtol(token, NULL, 16);
		token = strtok(NULL, ":");
		_bssid[3] = (byte)strtol(token, NULL, 16);
		token = strtok(NULL, ":");
		_bssid[2] = (byte)strtol(token, NULL, 16);
		token = strtok(NULL, ":");
		_bssid[1] = (byte)strtol(token, NULL, 16);
		token = strtok(NULL, ":");
		_bssid[0] = (byte)strtol(token, NULL, 16);
	}
	return _bssid;

}

int32_t EspDrv::getCurrentRSSI()
{
	LOGDEBUG(F("> getCurrentRSSI"));

    int ret=0;
	char buf[10];
	sendCmdGet(F("AT+CWJAP?"), F(",-"), F("\r\n"), buf, sizeof(buf));

	if (isDigit(buf[0])) {
      ret = -atoi(buf);
    }

    return ret;
}


uint8_t EspDrv::getScanNetworks()
{
    // Non-blocking state machine
    static enum { GS_IDLE, GS_SEND_CMD, GS_WAIT_RESP, GS_PARSE_ENTRY, GS_DONE, GS_FAIL } state = GS_IDLE;
    static unsigned long startTime = 0;
    static uint8_t ssidListNum = 0;
    static int idx = -1;

    switch (state) {
        case GS_IDLE:
            espEmptyBuf();
            LOGDEBUG(F("----------------------------------------------"));
            LOGDEBUG(F(">> AT+CWLAP"));
            espSerial->println(F("AT+CWLAP"));
            startTime = millis();
            ssidListNum = 0;
            state = GS_WAIT_RESP;
            return 0;

        case GS_WAIT_RESP:
            if (espSerial->available()) {
                idx = readUntil(0, "+CWLAP:(", false); // 0 timeout: just check buffer
                if (idx == NUMESPTAGS) {
                    state = GS_PARSE_ENTRY;
                } else if (millis() - startTime > 10000) {
                    LOGERROR(F("Scan networks: timeout waiting for response"));
                    state = GS_FAIL;
                }
            } else if (millis() - startTime > 10000) {
                LOGERROR(F("Scan networks: timeout waiting for response"));
                state = GS_FAIL;
            }
            return 0;

        case GS_PARSE_ENTRY:
            if (ssidListNum >= WL_NETWORKS_LIST_MAXNUM) {
                state = GS_DONE;
                return 0;
            }
            _networkEncr[ssidListNum] = espSerial->parseInt();
            readUntil(0, "\""); // discard , and " characters
            idx = readUntil(0, "\"", false);
            if (idx == NUMESPTAGS) {
                memset(_networkSsid[ssidListNum], 0, WL_SSID_MAX_LENGTH );
                ringBuf.getStrN(_networkSsid[ssidListNum], 1, WL_SSID_MAX_LENGTH-1);
            }
            readUntil(0, ","); // discard , character
            _networkRssi[ssidListNum] = espSerial->parseInt();
            idx = readUntil(0, "+CWLAP:(");
            if (idx == NUMESPTAGS) {
                ssidListNum++;
                state = GS_PARSE_ENTRY; // Parse next entry
            } else if (idx == -1) {
                state = GS_DONE;
            }
            return 0;

        case GS_DONE:
            LOGDEBUG1(F("---------------------------------------------- >"), ssidListNum);
            LOGDEBUG();
            state = GS_IDLE;
            return ssidListNum;

        case GS_FAIL:
            state = GS_IDLE;
            return (uint8_t)-1;
    }
    return 0;
}

bool EspDrv::getNetmask(IPAddress& mask) {
	LOGDEBUG(F("> getNetmask"));

	char buf[20];
	if (sendCmdGet(F("AT+CIPSTA?"), F("+CIPSTA:netmask:\""), F("\""), buf, sizeof(buf)))
	{
		mask.fromString (buf);
		return true;
	}

	return false;
}

bool EspDrv::getGateway(IPAddress& gw)
{
	LOGDEBUG(F("> getGateway"));

	char buf[20];
	if (sendCmdGet(F("AT+CIPSTA?"), F("+CIPSTA:gateway:\""), F("\""), buf, sizeof(buf)))
	{
		gw.fromString (buf);
		return true;
	}

	return false;
}

char* EspDrv::getSSIDNetoworks(uint8_t networkItem)
{
	if (networkItem >= WL_NETWORKS_LIST_MAXNUM)
		return NULL;

	return _networkSsid[networkItem];
}

uint8_t EspDrv::getEncTypeNetowrks(uint8_t networkItem)
{
	if (networkItem >= WL_NETWORKS_LIST_MAXNUM)
		return 0;

    return _networkEncr[networkItem];
}

int32_t EspDrv::getRSSINetoworks(uint8_t networkItem)
{
	if (networkItem >= WL_NETWORKS_LIST_MAXNUM)
		return 0;

    return _networkRssi[networkItem];
}

char* EspDrv::getFwVersion()
{
	LOGDEBUG(F("> getFwVersion"));

	fwVersion[0] = 0;

	sendCmdGet(F("AT+GMR"), F("SDK version:"), F("\r\n"), fwVersion, sizeof(fwVersion));

    return fwVersion;
}



bool EspDrv::ping(const char *host)
{
	LOGDEBUG(F("> ping"));

	int ret = sendCmd(F("AT+PING=\"%s\""), 8000, host);
	
	if (ret==TAG_OK)
		return true;

	return false;
}



// Start server TCP on port specified
bool EspDrv::startServer(uint16_t port, uint8_t sock)
{
	LOGDEBUG1(F("> startServer"), port);

	int ret = sendCmd(F("AT+CIPSERVER=%d,%d"), 1000, sock, port);

	return ret==TAG_OK;
}


bool EspDrv::startClient(const char* host, uint16_t port, uint8_t sock, uint8_t protMode)
{
	LOGDEBUG2(F("> startClient"), host, port);
	
	// TCP
	// AT+CIPSTART=<link ID>,"TCP",<remote IP>,<remote port>

	// UDP
	// AT+CIPSTART=<link ID>,"UDP",<remote IP>,<remote port>[,<UDP local port>,<UDP mode>]

	// for UDP we set a dummy remote port and UDP mode to 2
	// this allows to specify the target host/port in CIPSEND

	
	int ret = -1;
	if (protMode==TCP_MODE)
		ret = sendCmd(F("AT+CIPSTART=%d,\"TCP\",\"%s\",%u"), 5000, sock, host, port);
	else if (protMode==SSL_MODE)
	{
		// better to put the CIPSSLSIZE here because it is not supported before firmware 1.4
		sendCmd(F("AT+CIPSSLSIZE=4096"));
		ret = sendCmd(F("AT+CIPSTART=%d,\"SSL\",\"%s\",%u"), 5000, sock, host, port);
	}
	else if (protMode==UDP_MODE)
		ret = sendCmd(F("AT+CIPSTART=%d,\"UDP\",\"%s\",0,%u,2"), 5000, sock, host, port);

	return ret==TAG_OK;
}


// Start server TCP on port specified
void EspDrv::stopClient(uint8_t sock)
{
	LOGDEBUG1(F("> stopClient"), sock);

	sendCmd(F("AT+CIPCLOSE=%d"), 4000, sock);
}


uint8_t EspDrv::getServerState(uint8_t sock)
{
    return 0;
}



////////////////////////////////////////////////////////////////////////////
// TCP/IP functions
////////////////////////////////////////////////////////////////////////////



uint16_t EspDrv::availData(uint8_t connId)
{
    //LOGDEBUG(bufPos);

	// if there is data in the buffer
	if (_bufPos>0)
	{
		if (_connId==connId)
			return _bufPos;
		else if (_connId==0)
			return _bufPos;
	}


    int bytes = espSerial->available();

	if (bytes)
	{
		//LOGDEBUG1(F("Bytes in the serial buffer: "), bytes);
		if (espSerial->find((char *)"+IPD,"))
		{
			// format is : +IPD,<id>,<len>:<data>
			// format is : +IPD,<ID>,<len>[,<remote IP>,<remote port>]:<data>

			_connId = espSerial->parseInt();    // <ID>
			espSerial->read();                  // ,
			_bufPos = espSerial->parseInt();    // <len>
			espSerial->read();                  // "
			_remoteIp[0] = espSerial->parseInt();    // <remote IP>
			espSerial->read();                  // .
			_remoteIp[1] = espSerial->parseInt();
			espSerial->read();                  // .
			_remoteIp[2] = espSerial->parseInt();
            espSerial->read();                  // .
			_remoteIp[3] = espSerial->parseInt();
			espSerial->read();                  // ,
			_remotePort = espSerial->parseInt();     // <remote port>
			
			espSerial->read();                  // :

			LOGDEBUG();
			LOGDEBUG2(F("Data packet"), _connId, _bufPos);

			if(_connId==connId || connId==0)
				return _bufPos;
		}
	}
	return 0;
}


bool EspDrv::getData(uint8_t connId, uint8_t *data, bool peek, bool* connClose)
{
    // Static state variables to persist between calls
    static enum { GD_IDLE, GD_WAIT, GD_DONE, GD_FAIL } state = GD_IDLE;
    static long startMillis = 0;
    static uint8_t lastConnId = 0;

    if (state == GD_IDLE) {
        if (connId != _connId) return false;
        lastConnId = connId;
        startMillis = millis();
        *connClose = false;
        state = GD_WAIT;
    }

    if (state == GD_WAIT) {
        if (espSerial->available()) {
            if (peek) {
                *data = (char)espSerial->peek();
            } else {
                *data = (char)espSerial->read();
                _bufPos--;
            }

            if (_bufPos == 0) {
                // after the data packet a ",CLOSED" string may be received
                delay(5); // optional, can be removed for more responsiveness

                if (espSerial->available()) {
                    if (espSerial->peek() == 48 + lastConnId) { // '0' + connId
                        int idx = readUntil(500, ",CLOSED\r\n", false);
                        if (idx != NUMESPTAGS) {
                            LOGERROR(F("Tag CLOSED not found"));
                        }
                        LOGDEBUG();
                        LOGDEBUG(F("Connection closed"));
                        *connClose = true;
                    }
                }
            }
            state = GD_DONE;
        } else if (millis() - startMillis > 2000) {
            LOGERROR1(F("TIMEOUT:"), _bufPos);
            _bufPos = 0;
            _connId = 0;
            *data = 0;
            state = GD_FAIL;
        }
    }

    if (state == GD_DONE) {
        state = GD_IDLE;
        return true;
    }
    if (state == GD_FAIL) {
        state = GD_IDLE;
        return false;
    }
    return false;
}

/**
 * Receive the data into a buffer.
 * It reads up to bufSize bytes.
 * @return	received data size for success else -1.
 */
int EspDrv::getDataBuf(uint8_t connId, uint8_t *buf, uint16_t bufSize)
{
    // Static state variables to persist between calls
    static enum { GDBUF_IDLE, GDBUF_READING, GDBUF_DONE, GDBUF_FAIL, GDBUF_CONN_MISMATCH } state = GDBUF_IDLE;
    static uint16_t bytesRead = 0;
    static unsigned long startMillis = 0;
    //static uint8_t lastConnId = 0;
    static uint16_t lastBufSize = 0;
    static uint8_t* lastBuf = nullptr;

    switch (state) {
        case GDBUF_IDLE:
            if (connId != _connId) {
                LOGERROR(F("getDataBuf: connection ID mismatch"));
                state = GDBUF_CONN_MISMATCH;
                break;
            }
            lastBufSize = (_bufPos < bufSize) ? _bufPos : bufSize;
            bytesRead = 0;
            //lastConnId = connId;
            lastBuf = buf;
            startMillis = millis();
            state = GDBUF_READING;
            // fall through

        case GDBUF_READING:
            while (bytesRead < lastBufSize && espSerial->available()) {
                int c = espSerial->read();
                if (c == -1) {
                    LOGERROR(F("getDataBuf: serial read error"));
                    state = GDBUF_FAIL;
                    break;
                }
                lastBuf[bytesRead++] = (char)c;
                _bufPos--;
            }
            if (bytesRead == lastBufSize) {
                //CONSOLE.println("getDataBuf: success");
                state = GDBUF_DONE;
            } else if (millis() - startMillis > 1000) { // 1s timeout
                LOGERROR(F("getDataBuf: timeout"));
                state = GDBUF_FAIL;
            }
            break;

        case GDBUF_DONE: {
            int result = bytesRead;
            state = GDBUF_IDLE;
            return result;
        }

        case GDBUF_FAIL:
            state = GDBUF_IDLE;
            return -1;

        case GDBUF_CONN_MISMATCH:
            state = GDBUF_IDLE;
            return -1;
    }
    return 0; // Still in progress
}


bool EspDrv::sendData(uint8_t sock, const uint8_t *data, uint16_t len)
{
    static enum { SD_IDLE, SD_WAIT_PROMPT, SD_WRITE_DATA, SD_WAIT_SENDOK, SD_DONE, SD_FAIL } state = SD_IDLE;
    static unsigned long startTime = 0;
    static uint16_t bytesWritten = 0;
    static const uint8_t* lastData = nullptr;
    static uint16_t lastLen = 0;
    //static uint8_t lastSock = 0;

    switch (state) {
        case SD_IDLE:
            //lastSock = sock;
            lastData = data;
            lastLen = len;
            bytesWritten = 0;
            char cmdBuf[24];
            sprintf_P(cmdBuf, PSTR("AT+CIPSEND=%d,%u"), sock, len);
            espSerial->println(cmdBuf);
            startTime = millis();
            state = SD_WAIT_PROMPT;
            return false;

        case SD_WAIT_PROMPT: {
            int idx = -1;
            // Non-blocking: check for prompt '>''
            if (espSerial->available()) {
                idx = readUntil(0, (char *)">", false); // 0 timeout: just check buffer
                if (idx == NUMESPTAGS) {
                    state = SD_WRITE_DATA;
                } else if (millis() - startTime > 1000) {
                    LOGERROR(F("Data packet send error (1): prompt timeout"));
                    state = SD_FAIL;
                }
            } else if (millis() - startTime > 1000) {
                LOGERROR(F("Data packet send error (1): prompt timeout"));
                state = SD_FAIL;
            }
            return false;
        }

        case SD_WRITE_DATA:
            // Write as much as possible this call
            while (bytesWritten < lastLen && espSerial->availableForWrite()) {
                espSerial->write(lastData[bytesWritten++]);
            }
            if (bytesWritten == lastLen) {
                startTime = millis();
                state = SD_WAIT_SENDOK;
            }
            return false;

        case SD_WAIT_SENDOK: {
            int idx = -1;
            if (espSerial->available()) {
                idx = readUntil(0, nullptr, true); // 0 timeout: just check buffer for tags
                if (idx == TAG_SENDOK) {
                    state = SD_DONE;
                } else if (idx >= 0 && idx != TAG_SENDOK) {
                    LOGERROR(F("Data packet send error (2): wrong tag"));
                    state = SD_FAIL;
                } else if (millis() - startTime > 2000) {
                    LOGERROR(F("Data packet send error (2): sendok timeout"));
                    state = SD_FAIL;
                }
            } else if (millis() - startTime > 2000) {
                LOGERROR(F("Data packet send error (2): sendok timeout"));
                state = SD_FAIL;
            }
            return false;
        }

        case SD_DONE:
            state = SD_IDLE;
            return true;

        case SD_FAIL:
            state = SD_IDLE;
            return false;
    }
    return false;
}

// Overrided sendData method for __FlashStringHelper strings
bool EspDrv::sendData(uint8_t sock, const __FlashStringHelper *data, uint16_t len, bool appendCrLf)
{
    static enum { SDF_IDLE, SDF_WAIT_PROMPT, SDF_WRITE_DATA, SDF_WRITE_CRLF, SDF_WAIT_SENDOK, SDF_DONE, SDF_FAIL } state = SDF_IDLE;
    static unsigned long startTime = 0;
    static uint16_t bytesWritten = 0;
    static uint16_t lastLen = 0;
    static uint8_t lastSock = 0;
    static bool lastAppendCrLf = false;
    static PGM_P lastData = nullptr;

    switch (state) {
        case SDF_IDLE:
            lastSock = sock;
            lastData = reinterpret_cast<PGM_P>(data);
            lastLen = len;
            lastAppendCrLf = appendCrLf;
            bytesWritten = 0;
            {
                char cmdBuf[20];
                uint16_t len2 = len + 2 * appendCrLf;
                sprintf_P(cmdBuf, PSTR("AT+CIPSEND=%d,%u"), sock, len2);
                espSerial->println(cmdBuf);
            }
            startTime = millis();
            state = SDF_WAIT_PROMPT;
            return false;

        case SDF_WAIT_PROMPT: {
            int idx = -1;
            if (espSerial->available()) {
                idx = readUntil(0, (char *)">", false); // 0 timeout: just check buffer
                if (idx == NUMESPTAGS) {
                    state = SDF_WRITE_DATA;
                } else if (millis() - startTime > 1000) {
                    LOGERROR(F("FlashString send error (1): prompt timeout"));
                    state = SDF_FAIL;
                }
            } else if (millis() - startTime > 1000) {
                LOGERROR(F("FlashString send error (1): prompt timeout"));
                state = SDF_FAIL;
            }
            return false;
        }

        case SDF_WRITE_DATA:
            // Write as much as possible this call
            while (bytesWritten < lastLen && espSerial->availableForWrite()) {
                unsigned char c = pgm_read_byte(lastData + bytesWritten);
                espSerial->write(c);
                bytesWritten++;
            }
            if (bytesWritten == lastLen) {
                if (lastAppendCrLf) {
                    state = SDF_WRITE_CRLF;
                } else {
                    startTime = millis();
                    state = SDF_WAIT_SENDOK;
                }
            }
            return false;

        case SDF_WRITE_CRLF:
            if (espSerial->availableForWrite() >= 2) {
                espSerial->write('\r');
                espSerial->write('\n');
                startTime = millis();
                state = SDF_WAIT_SENDOK;
            }
            return false;

        case SDF_WAIT_SENDOK: {
            int idx = -1;
            if (espSerial->available()) {
                idx = readUntil(0, nullptr, true); // 0 timeout: just check buffer for tags
                if (idx == TAG_SENDOK) {
                    state = SDF_DONE;
                } else if (idx >= 0 && idx != TAG_SENDOK) {
                    LOGERROR(F("FlashString send error (2): wrong tag"));
                    state = SDF_FAIL;
                } else if (millis() - startTime > 2000) {
                    LOGERROR(F("FlashString send error (2): sendok timeout"));
                    state = SDF_FAIL;
                }
            } else if (millis() - startTime > 2000) {
                LOGERROR(F("FlashString send error (2): sendok timeout"));
                state = SDF_FAIL;
            }
            return false;
        }

        case SDF_DONE:
            state = SDF_IDLE;
            return true;

        case SDF_FAIL:
            state = SDF_IDLE;
            return false;
    }
    return false;
}

bool EspDrv::sendDataUdp(uint8_t sock, const char* host, uint16_t port, const uint8_t *data, uint16_t len)
{
    static enum { SDU_IDLE, SDU_WAIT_PROMPT, SDU_WRITE_DATA, SDU_WAIT_SENDOK, SDU_DONE, SDU_FAIL } state = SDU_IDLE;
    static unsigned long startTime = 0;
    static uint16_t bytesWritten = 0;
    static const uint8_t* lastData = nullptr;
    static uint16_t lastLen = 0;
    //static uint8_t lastSock = 0;
    static char lastHost[64];
    static uint16_t lastPort = 0;

    switch (state) {
        case SDU_IDLE:
            //lastSock = sock;
            strncpy(lastHost, host, sizeof(lastHost) - 1);
            lastHost[sizeof(lastHost) - 1] = 0;
            lastPort = port;
            lastData = data;
            lastLen = len;
            bytesWritten = 0;
            {
                char cmdBuf[80];
                snprintf(cmdBuf, sizeof(cmdBuf), "AT+CIPSEND=%d,%u,\"%s\",%u", sock, len, lastHost, lastPort);
                espSerial->println(cmdBuf);
            }
            startTime = millis();
            state = SDU_WAIT_PROMPT;
            return false;

        case SDU_WAIT_PROMPT: {
            int idx = -1;
            if (espSerial->available()) {
                idx = readUntil(0, (char *)">", false); // 0 timeout: just check buffer
                if (idx == NUMESPTAGS) {
                    state = SDU_WRITE_DATA;
                } else if (millis() - startTime > 1000) {
                    LOGERROR(F("UDP send error (1): prompt timeout"));
                    state = SDU_FAIL;
                }
            } else if (millis() - startTime > 1000) {
                LOGERROR(F("UDP send error (1): prompt timeout"));
                state = SDU_FAIL;
            }
            return false;
        }

        case SDU_WRITE_DATA:
            while (bytesWritten < lastLen && espSerial->availableForWrite()) {
                espSerial->write(lastData[bytesWritten++]);
            }
            if (bytesWritten == lastLen) {
                startTime = millis();
                state = SDU_WAIT_SENDOK;
            }
            return false;

        case SDU_WAIT_SENDOK: {
            int idx = -1;
            if (espSerial->available()) {
                idx = readUntil(0, nullptr, true); // 0 timeout: just check buffer for tags
                if (idx == TAG_SENDOK) {
                    state = SDU_DONE;
                } else if (idx >= 0 && idx != TAG_SENDOK) {
                    LOGERROR(F("UDP send error (2): wrong tag"));
                    state = SDU_FAIL;
                } else if (millis() - startTime > 2000) {
                    LOGERROR(F("UDP send error (2): sendok timeout"));
                    state = SDU_FAIL;
                }
            } else if (millis() - startTime > 2000) {
                LOGERROR(F("UDP send error (2): sendok timeout"));
                state = SDU_FAIL;
            }
            return false;
        }

        case SDU_DONE:
            state = SDU_IDLE;
            return true;

        case SDU_FAIL:
            state = SDU_IDLE;
            return false;
    }
}



void EspDrv::getRemoteIpAddress(IPAddress& ip)
{
	ip = _remoteIp;
}

uint16_t EspDrv::getRemotePort()
{
	return _remotePort;
}


////////////////////////////////////////////////////////////////////////////
// Utility functions
////////////////////////////////////////////////////////////////////////////



/*
* Sends the AT command and stops if any of the TAGS is found.
* Extract the string enclosed in the passed tags and returns it in the outStr buffer.
* Returns true if the string is extracted, false if tags are not found of timed out.
*/
bool EspDrv::sendCmdGet(const __FlashStringHelper* cmd, const char* startTag, const char* endTag, char* outStr, int outStrLen)
{
    static enum { SCG_IDLE, SCG_SEND, SCG_WAIT_START, SCG_WAIT_END, SCG_DONE, SCG_FAIL } state = SCG_IDLE;
    static int idx = IN_PROGRESS;
    static char* lastOutStr = nullptr;
    static int lastOutStrLen = 0;
    static const char* lastStartTag = nullptr;
    static const char* lastEndTag = nullptr;
    static const __FlashStringHelper* lastCmd = nullptr;

    switch (state) {
        case SCG_IDLE:
            espEmptyBuf();
            LOGDEBUG(F("----------------------------------------------"));
            LOGDEBUG1(F(">>"), cmd);
            espSerial->println(cmd);
            lastCmd = cmd;
            lastStartTag = startTag;
            lastEndTag = endTag;
            lastOutStr = outStr;
            lastOutStrLen = outStrLen;
            outStr[0] = 0;
            state = SCG_WAIT_START;
            return false;

        case SCG_WAIT_START:
            idx = readUntil(1000, lastStartTag);
            if (idx == NUMESPTAGS) {
                ringBuf.init();
                state = SCG_WAIT_END;
            } else if (idx >= 0 && idx < NUMESPTAGS) {
                LOGDEBUG1(F("No start tag found:"), idx);
                state = SCG_FAIL;
            } else if (idx == -1) {
                LOGWARN(F("No tag found"));
                state = SCG_FAIL;
            }
            return false;

        case SCG_WAIT_END:
            idx = readUntil(500, lastEndTag);
            if (idx == NUMESPTAGS) {
                ringBuf.getStrN(lastOutStr, strlen(lastEndTag), lastOutStrLen-1);
                readUntil(2000);
                state = SCG_DONE;
            } else if (idx == -1) {
                LOGWARN(F("End tag not found"));
                state = SCG_FAIL;
            }
            return false;

        case SCG_DONE:
            LOGDEBUG1(F("---------------------------------------------- >"), lastOutStr);
            LOGDEBUG();
            state = SCG_IDLE;
            return true;

        case SCG_FAIL:
            state = SCG_IDLE;
            return false;
    }
    return false;
}

bool EspDrv::sendCmdGet(const __FlashStringHelper* cmd, const __FlashStringHelper* startTag, const __FlashStringHelper* endTag, char* outStr, int outStrLen)
{
    char _startTag[strlen_P((char*)startTag)+1];
    strcpy_P(_startTag,  (char*)startTag);

    char _endTag[strlen_P((char*)endTag)+1];
    strcpy_P(_endTag,  (char*)endTag);

    return sendCmdGet(cmd, _startTag, _endTag, outStr, outStrLen);
}

/*
* Sends the AT command and returns the id of the TAG.
* Return -1 if no tag is found.
*/
int EspDrv::sendCmd(const __FlashStringHelper* cmd, int timeout)
{
    static enum { CMD_IDLE, CMD_SENT, CMD_WAIT } state = CMD_IDLE;
    static int result = IN_PROGRESS;
    static int lastTimeout = 0;
    static const __FlashStringHelper* lastCmd = nullptr;

    switch (state) {
        case CMD_IDLE:
            espEmptyBuf();
            LOGDEBUG(F("----------------------------------------------"));
            LOGDEBUG1(F(">>"), cmd);
            espSerial->println(cmd);
            lastCmd = cmd;
            lastTimeout = timeout;
            state = CMD_WAIT;
            result = IN_PROGRESS;
            return IN_PROGRESS;
        case CMD_WAIT:
            result = readUntil(lastTimeout);
            if (result != IN_PROGRESS) {
                LOGDEBUG1(F("---------------------------------------------- >"), result);
                LOGDEBUG();
                state = CMD_IDLE;
                return result;
            }
            return IN_PROGRESS;
    }
    return IN_PROGRESS;
}

/*
* Sends the AT command and returns the id of the TAG.
* The additional arguments are formatted into the command using sprintf.
* Return -1 if no tag is found.
*/
int EspDrv::sendCmd(const __FlashStringHelper* cmd, int timeout, ...)
{
    char cmdBuf[CMD_BUFFER_SIZE];

    va_list args;
    va_start (args, timeout);
    vsnprintf_P (cmdBuf, CMD_BUFFER_SIZE, (char*)cmd, args);
    va_end (args);

    espEmptyBuf();

    LOGDEBUG(F("----------------------------------------------"));
    LOGDEBUG1(F(">>"), cmdBuf);

    espSerial->println(cmdBuf);

    int idx = readUntil(timeout);

    LOGDEBUG1(F("---------------------------------------------- >"), idx);
    LOGDEBUG();

    return idx;
}

// Read from serial until one of the tags is found
// Returns:
//   the index of the tag found in the ESPTAGS array
//   -1 if no tag was found (timeout)
int EspDrv::readUntil(unsigned int timeout, const char* tag, bool findTags)
{
    static unsigned long start = 0;
    static int ret = -1;
    static bool initialized = false;

    if (!initialized) {
        ringBuf.reset();
        start = millis();
        ret = -1;
        initialized = true;
    }

    while (espSerial->available()) {
        char c = (char)espSerial->read();
        LOGDEBUG0(c);
        ringBuf.push(c);

        if (tag != NULL) {
            if (ringBuf.endsWith(tag)) {
                ret = NUMESPTAGS;
                initialized = false;
                return ret;
            }
        }
        if (findTags) {
            for (int i = 0; i < NUMESPTAGS; i++) {
                if (ringBuf.endsWith(ESPTAGS[i])) {
                    ret = i;
                    initialized = false;
                    return ret;
                }
            }
        }
    }

    if (millis() - start >= timeout) {
        LOGWARN(F(">>> TIMEOUT >>>"));
        initialized = false;
        return -1;
    }

    return IN_PROGRESS; // IN_PROGRESS
}

void EspDrv::espEmptyBuf(bool warn)
{
    char c;
    int i=0;
    while(espSerial->available() > 0)
    {
        c = espSerial->read();
        if (i>0 and warn==true)
            LOGDEBUG0(c);
        i++;
    }
    if (i>0 and warn==true)
    {
        LOGDEBUG(F(""));
        LOGDEBUG1(F("Dirty characters in the serial buffer! >"), i);
    }
}

// copied from Serial::timedRead
int EspDrv::timedRead()
{
  unsigned int _timeout = 1000;
  int c;
  long _startMillis = millis();
  do
  {
    c = espSerial->read();
    if (c >= 0) return c;
  } while(millis() - _startMillis < _timeout);

  return -1; // -1 indicates timeout
}

EspDrv espDrv;
