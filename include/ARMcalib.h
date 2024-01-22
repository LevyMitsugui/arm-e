//Base servo:
#define min1 500
#define max1 2500
//Shoulder Servo:
#define min2 1100//1442
#define max2 2307
//Elbow Servo:
#define min3 850//963
#define max3 1853//1833
//Grabber Servo:
#define min4 500
#define max4 1950

//links length in millimeters:
#define CAL_L_0 (float)3
#define CAL_L_1 (float)16
#define CAL_L_2 (float)80
#define CAL_L_3 (float)80
#define CAL_L_4 (float)80
#define CAL_L_5 (float)75

#define CAL_L_3I (float)35
#define CAL_L_3O (float)35

#define CAL_D_5 (float)8
#define CAL_H_1 (float)60

#define CAL_Hub_Diameter (float)80.5
#define CAL_SUP_CAP_FACTOR (float)0.84
#define CAL_SUP_CAP (float) (CAL_L_2+CAL_L_3)*CAL_SUP_CAP_FACTOR

#define CAL_Z_SUP_CAP (float) 130
#define CAL_Z_BOT_CAP (float) 20

#define POS_MARGIN (float) 0.5
#define MAX_SPEED (float) 150

#define TOF_Z_CLEARANCE(float) 77 // z grabbar hight in witch it clears the tof view
#define R_GRABBER_OFFSET (float) 40