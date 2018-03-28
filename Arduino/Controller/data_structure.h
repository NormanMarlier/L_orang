#ifndef data_structure_h
#define data_structure_h


/* ========================================================= *\
 * This structure is used to retain the angular values       *
 * of the servomotors                                        *
 *                                                           *
 * FIELDS :                                                  *
 * angle_1 : float                                           *
 * The value of the angle of the servo_1                     *
 *                                                           *
 * angle_2 : float                                           *
 * The value of the angle of the servo_2                     *
 *                                                           *
 * angle_3 : float                                           *
 * The value of the angle of the servo_3                     *
 *                                                           *
\* ========================================================= */
typedef struct
{
    float angle_1;
    float angle_2;
    float angle_3;
    
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
