/*
FILENAME...	serialIO.h
USAGE...	.

Version:	1.5
Modified By:	rivers
Last Modified:	2003/09/17 16:43:05
*/
 
/*****************************************************************
                          COPYRIGHT NOTIFICATION
*****************************************************************

(C)  COPYRIGHT 1993 UNIVERSITY OF CHICAGO

This software was developed under a United States Government license
described on the COPYRIGHT_UniversityOfChicago file included as part
of this distribution.
**********************************************************************/

/*
 *      Original Author: Mark Rivers
 *
 * Modification Log:
 * -----------------
 */

#ifndef	INCserialIOh
#define	INCserialIOh 1

#include "Message.h"

#ifdef __cplusplus
class serialIO
{
public:
    serialIO(int, char *, bool *);
    int serialIOSend(char const *, int, int);
    int serialIORecv(char *, int, char *, int);
    int serialIOSendRecv(char const *, int, char *, int, char *, int);
    static void serialIOCallback(Message *, void *);
private:
    MessageClient* pMessageClient;
    epicsMessageQueue *msgQId;
};
#else  /* For C just define serialInfo as a dummy structure since it can't
          understand the include files which define what it really is */
void *serialIOInit(int, char *);
int serialIOSend(void *, char const *, int, int);
int serialIORecv(void *, char *, int, char *, int);
int serialIOSendRecv(void *, const char *, int, char *, int, char *, int);
#endif

#endif
