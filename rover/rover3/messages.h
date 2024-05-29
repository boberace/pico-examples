
#ifndef MESSAGES_H
#define MESSAGES_H

typedef struct {
    int direction;
    int speed;
    int enable;
} ControlData;

extern ControlData control_data;



#endif // MESSAGES_H
