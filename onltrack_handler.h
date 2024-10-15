#ifndef ROBOTARM_ONLTRACK_HANDLER_H
#define ROBOTARM_ONLTRACK_HANDLER_H

typedef struct{
    char Command;
    char char_dummy[3];
    int State;
    int Count;
    int int_dummy;
    double coord[6];
} Hyundai_Data_t;         //ToDo: rename?


typedef struct {
    double x;
    double y;
    double z;
    double rotx;
    double roty;
    double rotz;
} Cartesian_Pos_t;


void Onltrack_AnswerHandle(const Hyundai_Data_t* eePos_worldFrame, long buflen);

#endif //ROBOTARM_ONLTRACK_HANDLER_H
