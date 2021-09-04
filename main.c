/*---------------------------*\

    II1300 Ingenjörsmetodik
    HT20

    LEGO ROBOT
    ev3dev-c

    GRUPP 10
    Johan, Ville, Caroline

    All kod är skriven av Johan,
    förutom markerade undantag.

\*---------------------------*/

#include <stdio.h>
#include <string.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"

#include <unistd.h>

#define SCANS 30
#define SCANS_HD 21
const int HD_SCAN_ANGLE = 20;

const int TURNING_SPEED_TINY = 40;
const int TURNING_SPEED_LOW = 120;
const int TURNING_SPEED_MEDIUM = 200;

//
//  MOTOR DATA STRUCTURE
//
struct Motor{
    char type[50];
    char port[50];
    int sn;
    int max_speed;
};
struct Motor motorRight;
struct Motor motorLeft;
struct Motor motorLift;

void printMotor(struct Motor motor){
    printf("  type = %s\n", motor.type);
    printf("  port = %s\n", motor.port);
    printf("  sn   = %d\n", motor.sn);
    printf("  max_speed = %d\n", motor.max_speed);
}

//
//  MOTOR BASIC CONTROL
//
void spin_base(int sn, int spd){
    set_tacho_ramp_up_sp(sn, 0);
    set_tacho_ramp_down_sp(sn, 0);
    set_tacho_polarity_inx(sn, TACHO_NORMAL);
    set_tacho_stop_action_inx(sn, TACHO_HOLD);
}
void spin_deg(struct Motor motor, int deg, int spd){
    int sn = motor.sn;
    spd = spd < motor.max_speed ? spd : motor.max_speed;
    spin_base(sn, spd);
    set_tacho_speed_sp(sn, spd);
    set_tacho_position_sp(sn, deg);
    set_tacho_command_inx(sn, TACHO_RUN_TO_REL_POS);
}
void spin_time(struct Motor motor, int dir, float sec, int spd){
    int sn = motor.sn;
    spd = spd < motor.max_speed ? spd : motor.max_speed;
    int msecs = sec * 1000;
    spin_base(sn, spd);
    if(dir <= 0) set_tacho_polarity_inx(sn, TACHO_INVERSED);
    set_tacho_speed_sp(sn, spd);
    set_tacho_time_sp(sn, msecs);
    set_tacho_command_inx(sn, TACHO_RUN_TIMED);
}
void spin_forever(struct Motor motor, int dir, int spd){
    int sn = motor.sn;
    spd = spd < motor.max_speed ? spd : motor.max_speed;
    spin_base(sn, spd);
    if(dir <= 0) set_tacho_polarity_inx(sn, TACHO_INVERSED);
    set_tacho_speed_sp(sn, spd);
    set_tacho_command_inx(sn, TACHO_RUN_FOREVER);
}
void stop(struct Motor motor){
    int sn = motor.sn;
    set_tacho_command_inx(sn, TACHO_STOP);
}
void reset(struct Motor motor){
    int sn = motor.sn;
    set_tacho_command_inx(sn, TACHO_RESET);
}
void wait(struct Motor motor){
    int sn = motor.sn;
    FLAGS_T state;
    do{
        sleep(0.05);
        get_tacho_state_flags(sn, &state);
    } while (state != TACHO_HOLDING);
}

//
//  SENSOR VALUES
//
int sensorUS_sn;
int sensorGyro_sn;
int sensorButton_sn;

int get_value(int sensor_sn){
    int val;
    get_sensor_value(0, sensor_sn, &val);
    return val;
}
int get_value_us(){int val = get_value(sensorUS_sn); if(val == 326) val = 2550; return val;}
int get_value_gyro(){int val = get_value(sensorGyro_sn); return val;}
int get_value_button(){int val = get_value(sensorButton_sn); return val;}

void wait_until_button(){
    printf("Waiting for button to be pressed\n");
    do {} while (get_value_button() == 0);
    sleep(0.1);
}

//
//  ROBOT DRIVING
//
const float WHEEL_CIRCUMF = 17.6; // cm
int wheel_cmps_to_degps(int cmps){
    // Converts a speed from cm/s (user-friendly) to deg/s (for motor).
    float one_cm_degs = 360.0 / WHEEL_CIRCUMF; // deg
    return cmps * one_cm_degs; // deg/s
}
void go_forward_cm(float cm, int spd){
    float laps = cm / WHEEL_CIRCUMF; // laps
    int deg = laps * 360; // deg
    int degspd = wheel_cmps_to_degps(spd);
    printf("Going: %.2f cm, %d deg, %d cm/s, %.2f laps\n", cm, deg, spd, laps);
    spin_deg(motorRight, deg, degspd);
    spin_deg(motorLeft, deg, degspd);
    wait(motorLeft);
    wait(motorRight);
}
void go_back_cm(float cm, int spd){go_forward_cm(-cm, spd);}
int drift_control(int dir, int target_dir, int drifting, int degspd){
    // SIDE DRIFT PROTECTION
    dir = get_value_gyro();
    if(dir == target_dir){ // back to normal speeds
        if(drifting){
            printf("Drift resolved.\n");
            spin_forever(motorRight, 1, degspd);
            spin_forever(motorLeft, 1, degspd);
            drifting = 0;
        }
    }
    else{
        drifting = 1;
        printf("Drift detected! Correcting...\n");
        if(dir > target_dir){ // drifting right
            spin_forever(motorLeft, 1, degspd * 0.75);
        }
        else{ // drifting left
            spin_forever(motorRight, 1, degspd * 0.75);
        }
    }
    return drifting;
}
void go_to_distance_from_wall(float cm_to_wall, int spd){
    int degspd = wheel_cmps_to_degps(spd);
    int degspd_slower = degspd > 150 ? 150 : degspd / 2;
    printf("Going until wall is %.2f cm away @ %d cm/s (%d deg/s)\n", cm_to_wall, spd, degspd);
    if(cm_to_wall < get_value_us() / 10.0){ // forward
        int dir, target_dir = get_value_gyro();
        int drifting = 0;
        spin_forever(motorRight, 1, degspd);
        spin_forever(motorLeft, 1, degspd);
        do {
            drifting = drift_control(dir, target_dir, drifting, degspd);
        } while (get_value_us() / 10.0 > cm_to_wall + spd / 3); // the wall is .33s away
        printf("Ramping down to %d deg/s\n", degspd_slower);
        // decrease speed the last bit
        spin_forever(motorRight, 1, degspd_slower);
        spin_forever(motorLeft, 1, degspd_slower);
        do {} while (get_value_us() / 10.0 > cm_to_wall); // the wall is at the right distance
    }
    ///// DENNA DEL SKRIVEN AV VILLE /////
    else { // backward
      spin_forever(motorRight, -1, degspd_slower);
      spin_forever(motorLeft, -1, degspd_slower);
      do {} while (get_value_us() / 10.0 < cm_to_wall); // the wall is at the right distance
    }
    ///// --- /////
    stop(motorRight);
    stop(motorLeft);
}

//
//  ROBOT TURNING CONTROL
//
void turn_forever(int dir, int spd){
    printf("Turning until something stops me :) direction: %d spd: %d\n", dir, spd);
    spin_forever(motorRight, -dir, spd);
    spin_forever(motorLeft, dir, spd);
}
void turn_towards(int target_rot, int spd, int precise){
    int init_rot = get_value_gyro();
    if(init_rot == target_rot)
        return; // skip turning if already there
    int deg = target_rot - init_rot;
    printf("Turning from %d until I reach %d @ %d wheel deg/s\n", init_rot, target_rot, spd);
    turn_forever(deg, spd);
    if(deg > 0) // clockwise
        do {} while (get_value_gyro() < target_rot);
    else        // counter-clockwise
        do {} while (get_value_gyro() > target_rot);
    printf("Done turning\n");
    stop(motorRight);
    stop(motorLeft);
    if(precise){
        wait(motorLeft);
        wait(motorRight);
        if(get_value_gyro() != target_rot){
            printf("Overturned! Turning back...\n");
            turn_towards(target_rot, spd / 2, precise);
        }
    }
}
void turn_deg(int deg, int spd, int precise){
    int init_rot = get_value_gyro();
    int target_rot = init_rot + deg;
    turn_towards(target_rot, spd, precise);
}

//
//  LIFT MOTOR CONTROL
//
void lift(){
    printf("Lifting fork\n");
    spin_time(motorLift, 1, 0.8, 200);
    wait(motorLift);
}
void drop(){
    printf("Dropping load\n");
    spin_deg(motorLift, -55, 500);
    wait(motorLift);
}

//
//  NAVIGATION
//
int scan_surroundings(int steps, int steplength, int start_rot, int hd){
    static int surr[360][2];
    int i;
    printf("Scanning %d times, starting from %d deg, %d deg per step\n", steps, start_rot, steplength);
    if(hd)
        turn_towards(start_rot, TURNING_SPEED_TINY, hd);
    else
        turn_towards(start_rot, TURNING_SPEED_LOW, hd);
    for(i = 0; i < steps; i++){
        sleep(0.1);
        // rotation and distance to object is saved
        surr[i][0] = get_value_gyro();
        surr[i][1] = get_value_us();
        if(i < steps - 1) // turn between all scans, not after the last one
            turn_deg(steplength, TURNING_SPEED_TINY, hd);
    }
    // Print scanned values for debugging
    printf("Scan result:\nangle:   ");
    for(i = 0; i < steps; i++)
        printf("%5d", surr[i][0]);
    printf("\ndistance:");
    for(i = 0; i < steps; i++)
        printf("%5d", surr[i][1]);
    printf("\n");

    // determine which direction from scan has the closest distance
    int min_val = 20000, min_idx, distance;
    for(i = 0; i < steps; i++){
        distance = surr[i][1];
        if(distance < min_val){
            min_val = distance;
            min_idx = i;
        }
    }
    int dir = surr[min_idx][0]; // direction to the closest wall
    printf("Scan determined %d is the direction to the wall\n", dir);
    return dir;
}
int optimize_dir(int dir){
    int init_rot = get_value_gyro();
    int diff = (dir - init_rot) % 360;
    while(diff < -180) diff += 360;
    while(diff > 180) diff -= 360;
    return init_rot + diff;
}

//
//  HIGH-LEVEL CONTROL
//
void goto_closest_wall(int offset, float cm_to_wall, int spd){
    // Look for where to go
    int dir = scan_surroundings(SCANS, 360 / SCANS, get_value_gyro(), 0) + offset;
    // Check if a shortcut is availible
    dir = optimize_dir(dir);
    // Scan more carefully
    dir = scan_surroundings(SCANS_HD, 2 * HD_SCAN_ANGLE / (SCANS_HD-1), dir - HD_SCAN_ANGLE, 1) + offset;
    // Go there
    turn_towards(dir, TURNING_SPEED_LOW, 1);
    go_to_distance_from_wall(cm_to_wall, spd);
}
void path(int N){
    int angle = N % 2 == 0 ? -90 : 90;
    int wall_offset = N > 2 ? 180 : 0;
    lift();
    wait_until_button();
    sleep(1);
    int dir = scan_surroundings(SCANS, 360 / SCANS, get_value_gyro(), 0);
    dir = optimize_dir(dir);
    dir = scan_surroundings(SCANS_HD, 2 * HD_SCAN_ANGLE / (SCANS_HD-1), dir - HD_SCAN_ANGLE, 1);
    turn_towards(dir + angle, TURNING_SPEED_LOW, 1);
    go_forward_cm(250, 15);
    turn_towards(optimize_dir(dir + wall_offset), TURNING_SPEED_LOW, 1);
    go_to_distance_from_wall(45, 12);
    sleep(1);
    drop();
    sleep(1);
    go_back_cm(20, 10);
    lift();
}


void line(){printf("-----------\n");}

int main(int argc, char *argv[]){
    printf("Grupp 10 robot\n");

    if(ev3_init() == -1) return 1;
    ev3_tacho_init();
    ev3_sensor_init();

    // FIND MOTORS
    char s[256]; // buffer for fetching values
    char type[50];
    char port[50];
    int max_speed;
    printf("Tacho motors detected:\n");
    for(int i = 0; i < DESC_LIMIT; i++){
        if(ev3_tacho[i].type_inx != TACHO_TYPE__NONE_){
            line();
            strcpy(type, ev3_tacho_type(ev3_tacho[i].type_inx));
            strcpy(port, ev3_tacho_port_name(i, s));
            get_tacho_max_speed(i, &max_speed);
            struct Motor tmp;
            strcpy(tmp.type, type);
            strcpy(tmp.port, port);
            tmp.sn = i;
            tmp.max_speed = max_speed;
            printMotor(tmp);
            if     (!strcmp(port, "ev3-ports:outB")){
                printf("Using motor on port B\n");
                motorRight = tmp;
            }
            else if(!strcmp(port, "ev3-ports:outC")){
                printf("Using motor on port C\n");
                motorLeft = tmp;
            }
            else if(!strcmp(port, "ev3-ports:outD")){
                printf("Using motor on port D\n");
                motorLift = tmp;
            }
        }
    }
    line();

    // FIND SENSORS
    int val;
    uint32_t n, i, ii;
    printf("Sensors detected:\n");
    for(int i = 0; i < DESC_LIMIT; i++){
        if(ev3_sensor[i].type_inx != TACHO_TYPE__NONE_){
            line();
            strcpy(type, ev3_sensor_type(ev3_tacho[i].type_inx));
            strcpy(port, ev3_sensor_port_name(i, s));
            printf("  type = %s\n", type);
            printf("  port = %s\n", port);
            printf("  sn   = %d\n", i);
            if(get_sensor_modes(i, s, sizeof(s))){
                printf("  modes = %s\n", s);
            }
            if(get_sensor_mode(i, s, sizeof(s))){
                printf("  mode = %s\n", s);
            }
            if(get_sensor_num_values(i, &n)){
                for(ii = 0; ii < n; ii++){
                    if(get_sensor_value(ii, i, &val)){
                        printf("  value%d = %d\n", ii, val);
                    }
                }
            }
            if     (!strcmp(port, "ev3-ports:in2")){
                printf("Using sensor on port 2\n");
                sensorUS_sn = i;
            }
            else if(!strcmp(port, "ev3-ports:in1")){
                printf("Using sensor on port 1\n");
                sensorGyro_sn = i;
            }
            else if(!strcmp(port, "ev3-ports:in3")){
                printf("Using sensor on port 3\n");
                sensorButton_sn = i;
            }
        }
    }
    line();

    // EXECUTE SEQUENCE OF EVENTS
    if(argc >= 2){
        for(int i = 1; i < argc; i++){
            if(!strcmp(argv[i], "none")){}
            else if(!strcmp(argv[i], "go")){
                go_forward_cm(10, 10);
            }
            else if(!strcmp(argv[i], "90")){
                turn_deg(90, TURNING_SPEED_MEDIUM, 1);
            }
            else if(!strcmp(argv[i], "-90")){
                turn_deg(-90, TURNING_SPEED_MEDIUM, 1);
            }
            else if(!strcmp(argv[i], "180")){
                turn_deg(180, TURNING_SPEED_MEDIUM, 1);
            }
            else if(!strcmp(argv[i], "-180")){
                turn_deg(-180, TURNING_SPEED_MEDIUM, 1);
            }
            else if(!strcmp(argv[i], "360")){
                turn_deg(360, TURNING_SPEED_MEDIUM, 1);
            }
            else if(!strcmp(argv[i], "backandforth")){
                go_to_distance_from_wall(30, 10);
                turn_deg(180, TURNING_SPEED_LOW, 1);
                go_to_distance_from_wall(30, 10);
                turn_deg(-180, TURNING_SPEED_LOW, 1);
            }
            else if(!strcmp(argv[i], "iamspeed")){
                go_to_distance_from_wall(50, 50);
            }
            else if(!strcmp(argv[i], "dance")){
                for(int d = 0; d < 4; d++){
                    go_forward_cm(10, 10);
                    go_forward_cm(10, 10);
                    go_back_cm(10, 10);
                    go_back_cm(10, 10);
                    turn_deg(270, 1000, 0);
                }
            }
            else if(!strcmp(argv[i], "scan")){
                scan_surroundings(SCANS, 360 / SCANS, get_value_gyro(), 0);
            }
            else if(!strcmp(argv[i], "scanhd")){
                scan_surroundings(SCANS_HD, HD_SCAN_ANGLE / (SCANS_HD-1), get_value_gyro(), 1);
            }
            else if(!strcmp(argv[i], "wall")){
                go_to_distance_from_wall(30, 20);
            }
            else if(!strcmp(argv[i], "closestwall")){
                goto_closest_wall(0, 55, 15);
            }
            else if(!strcmp(argv[i], "lift")){
                lift();
            }
            else if(!strcmp(argv[i], "drop")){
                drop();
            }
            else if(!strcmp(argv[i], "wait4book")){
                wait_until_button();
            }
            else if(!strcmp(argv[i], "sleep1")){
                sleep(1);
            }
            else if(!strcmp(argv[i], "back20")){
                go_back_cm(20, 20);
            }
            else if(!strcmp(argv[i], "path1")){
                path(1);
            }
            else if(!strcmp(argv[i], "path2")){
                path(2);
            }
            else if(!strcmp(argv[i], "path3")){
                path(3);
            }
            else if(!strcmp(argv[i], "path4")){
                path(4);
            }
        }
        // Stop all motors after instructions are done, but keep their state
        stop(motorLeft);
        stop(motorRight);
        stop(motorLift);
    }
    // No args -> reset all
    else{
        reset(motorLeft);
        reset(motorRight);
        reset(motorLift);
    }

    ev3_uninit();
    return 0;
}


