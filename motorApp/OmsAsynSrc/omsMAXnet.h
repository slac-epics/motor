/*
FILENAME...     omsMAXnet.h
USAGE...        Pro-Dex OMS MAXnet asyn motor controller support

Version:        $Revision: 1.3 $
Modified By:    $Author: ernesto $
Last Modified:  $Date: 2013/07/09 15:27:30 $
HeadURL:        $URL$
*/

/*
 *  Created on: 10/2010
 *      Author: eden
 */

#ifndef OMSMAXNET_H_
#define OMSMAXNET_H_

#include "omsBaseController.h"

class omsMAXnet : public omsBaseController {
public:
    omsMAXnet(const char* , int , const char*, const char*, int , int );
    static void asynCallback(void*, asynUser*, char *, size_t, int);
    int portConnected;
    int notificationCounter;
    epicsMutex* notificationMutex;
    epicsEventWaitStatus waitInterruptible(double);
//    asynStatus sendReceive(const char *, char *, unsigned int);
    asynStatus sendReceive(const char *, char *, unsigned int , size_t *);
    asynStatus sendOnly(const char *);
    virtual asynStatus flushQueues() {return asynSuccess;};

private:
    int isNotification (char *);
    asynUser* pasynUserSerial;
    asynUser* pasynUserSyncIOSerial;
    asynOctet *pasynOctetSerial;
    void* octetPvtSerial;
    char* serialPortName;
    double timeout;
};

#endif /* OMSMAXNET_H_ */
