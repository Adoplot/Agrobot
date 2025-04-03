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
    double x            {0};
    double y            {0};
    double z            {0};
    double rotx         {0};
    double roty         {0};
    double rotz         {0};
} Cartesian_Pos_t;


typedef struct{
    int id              {0};
    bool isReachable    {false};
    double x1  {0};
    double y1  {0};
    double z1  {0};
    double x2  {0};
    double y2  {0};
    double z2  {0};
} Target_Parameters_t;   //ToDo: PASHA - this is temporary struct, keep the name, keep x1,y1,z1,x2,y2,z2.


void Onltrack_AnswerHandle(const Hyundai_Data_t* eePos_worldFrame, long buflen);

#endif //ROBOTARM_ONLTRACK_HANDLER_H
