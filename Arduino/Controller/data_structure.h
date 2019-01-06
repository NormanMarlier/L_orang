#ifndef data_structure_h
#define data_structure_h

// Born for angles
#define ANGLE_1_MIN 0
#define ANGLE_1_MAX 180
#define ANGLE_2_MIN 80
#define ANGLE_2_MAX 150
#define ANGLE_3_MIN 80
#define ANGLE_3_MAX 135

/* ========================================================= *\
 * This structure is used to retain the angular values       *
 * of the servomotors                                        *
 *                                                           *
 * FIELDS :                                                  *
 * angle_1 : int                                             *
 * The value of the angle of the servo_1                     *
 *                                                           *
 * angle_2 : int                                             *
 * The value of the angle of the servo_2                     *
 *                                                           *
 * angle_3 : int                                             *
 * The value of the angle of the servo_3                     *
 *                                                           *
 * angle_4 : int                                             *
 * The value of the angle of the servo_4                     *
 *                                                           *
\* ========================================================= */
typedef struct
{
    int angle_1;
    int angle_2;
    int angle_3;
    int angle_4;
    
}Position;


/* ======================================================= *\
 * The enumeration describes the state of the grippper     *
 *                                                         *
 * TYPE :                                                  *
 * - OPEN : The gripper is fully opened                    *
 * - CLOSE : The gripper is fully closed                   *
 * - GRABBING : The gripper has a object                   *
 *                                                         *
\* ======================================================= */
typedef enum
{
    OPEN=180, CLOSE=0, GRABBING
    
}GRIPPER_STATE;


/* ========================================================= *\
 * This structure is used to retain the angular values       *
 * of the servomotors                                        *
 *                                                           *
 * FIELDS :                                                  *
 * pos : Position                                            *
 * The position of the robot                                 *
 *                                                           *
 * gripper : GRIPPER_STATE                                   *
 * The state of the gripper                                  *
 *                                                           *
\* ========================================================= */
typedef struct
{
    Position pos;
    GRIPPER_STATE gripper_state;    
    
}Robot;

#endif /*data_structure.h */
