/* 
 * File:   PowerBotClient.h
 * Author: Andoni Aguirrezabal
 *
 * Created on August 19, 2014, 4:09 PM
 */

#ifndef POWERBOTCLIENT_H
#define	POWERBOTCLIENT_H

#include "Aria.h"
#include "ArNetworking.h"

class PowerBotClient {
public:
    PowerBotClient();
    virtual ~PowerBotClient();

public:
    ///Connects to the PowerBot
    bool connect(void);
    bool getRunningWithLock(void);
    void moveTo(float posX, float posY);
    void rotateTo(float theta);
    void requestUpdate(void);
    void transformPoints(float &outX, float &outY);

private:
    void handleOutputNumbers(ArNetPacket *packet);

private:
    ArClientBase pbClient;
    ArFunctor1C<PowerBotClient, ArNetPacket *> pbOutputNumbersCB;

public:
    double pbVoltage, pbX, pbY, pbTh, pbVel, pbRotVel, pbLatVel, pbTemperature;
};

#endif	/* POWERBOTCLIENT_H */
