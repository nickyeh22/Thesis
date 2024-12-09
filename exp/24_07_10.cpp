// NTUCar.cpp : Defines the entry point for the console application.
//

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <thread>
#include <conio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include "Library/lib.hpp"
#include "innoAGV.h"
#include "stdafx.h"
#include <cstring>
#include <ctime>
#include <fstream>
#include <stdlib.h>
#include <windows.h>

using namespace std;
using namespace cv;

innoAGV myRacer;
//#define _MY_DEBUG_
#ifndef _MY_DEBUG_
#define DEF_CAR_IP  "192.168.0.150"
#else
#define DEF_CAR_IP  "192.168.1.4"
#endif
#define ROTATE_ANGLE    600
#define LOW_SPEED       600
#define HIGH_SPEED      1200
#define PI acos(-1)
#define INF DBL_MAX

// Define Membership Functions (MFs)
#define NB -2
#define NS -1
#define ZO 0
#define PS 1
#define PM 2
#define PB 3
#define PVB 4


void reverse_arr(double** arr, int sz) 
{
    double* temp;
    for (int i = 0; i < sz / 2; i++) {
        temp = arr[i];
        arr[i] = arr[sz - i - 1];
        arr[sz - i - 1] = temp;
    }
}

double func(double t, int* obs_pos, double vhc_hd, double c1, double c2, double del_x, double del_y, double beta, double R, double r_v, double r_o, double vhc_hd_new, int turning) 
{
    double x_pos, y_pos, phi, alpha;

    if (fabs(tan(vhc_hd)) < tan(PI / 4)) {
        x_pos = t + del_x;
        y_pos = (tan(vhc_hd) * t + c1) + del_y;
    }
    else {
        x_pos = (1 / tan(vhc_hd) * t + c2) + del_x;
        y_pos = t + del_y;
    }

    phi = atan2((obs_pos[1] - y_pos), (obs_pos[0] - x_pos));
    if ((obs_pos[1] - y_pos) < 0) {   // Third and fourth quadrants
        phi = 2 * PI + phi;
    }
    else if ((obs_pos[0] - x_pos) == 0 && (obs_pos[1] - y_pos) > 0) {
        phi = PI / 2;
    }
    else if ((obs_pos[0] - x_pos) == 0 && (obs_pos[1] - y_pos) < 0) {
        phi = 3 * PI / 2;
    }

    if (turning == 1) {      // Right turn
        alpha = PI / 2 + beta + (phi - vhc_hd_new);
    }
    else {                 // Left turn
        alpha = PI / 2 + beta - (phi - vhc_hd_new);
    }

    double y = sqrt(pow((obs_pos[1] - y_pos), 2) + pow((obs_pos[0] - x_pos), 2))
        - (sqrt(pow(R * cos(alpha), 2) + 2 * R * (r_v + r_o) + pow((r_v + r_o), 2)) + R * cos(alpha));

    return y;
}

double obs_ext_radi(double* vhc_pos, double vhc_hd, int obs_idx, int** obs_pos, double* obs_sz, double* turning_coeff) 
{
    double back_dist = 123.;          //往後拉的距離
    double r_o = obs_sz[obs_idx];
    double r_v = turning_coeff[0];
    double psi_rate = turning_coeff[1];
    double v = turning_coeff[2];
    double L = turning_coeff[3];
    double L_r = turning_coeff[4];
    double R = turning_coeff[5];
    double beta = turning_coeff[6];

    // Calculation for turning positions
    double vhc_hd_r = -(v / psi_rate / L) * log(fabs(1.0 / cos(-psi_rate * 2))) + vhc_hd;
    double vhc_hd_l = (v / psi_rate / L) * log(fabs(1.0 / cos(psi_rate * 2))) + vhc_hd;

    double del_x_r = 0.0;
    double del_y_r = 0.0;
    double del_x_l = 0.0;
    double del_y_l = 0.0;

    // Integration for del_x and del_y
    for (double t = 0; t <= 2; t += 0.001) {
        double x_diff_r = v * cos(atan(L_r / L * tan(-psi_rate * t))
            + v / (-psi_rate) / L * cos(L_r / L * tan(-psi_rate * t))
            * log(fabs(1.0 / cos(-psi_rate * t)))
            + vhc_hd);
        double y_diff_r = v * sin(atan(L_r / L * tan(-psi_rate * t))
            + v / (-psi_rate) / L * cos(L_r / L * tan(-psi_rate * t))
            * log(fabs(1.0 / cos(-psi_rate * t)))
            + vhc_hd);
        double x_diff_l = v * cos(atan(L_r / L * tan(psi_rate * t))
            + v / (psi_rate) / L * cos(L_r / L * tan(psi_rate * t))
            * log(fabs(1.0 / cos(psi_rate * t)))
            + vhc_hd);
        double y_diff_l = v * sin(atan(L_r / L * tan(psi_rate * t))
            + v / (psi_rate) / L * cos(L_r / L * tan(psi_rate * t))
            * log(fabs(1.0 / cos(psi_rate * t)))
            + vhc_hd);

        del_x_r += x_diff_r * 0.001;
        del_y_r += y_diff_r * 0.001;

        del_x_l += x_diff_l * 0.001;
        del_y_l += y_diff_l * 0.001;
    }

    double c1 = vhc_pos[1] - tan(vhc_hd) * vhc_pos[0];
    double c2 = vhc_pos[0] - 1.0 / tan(vhc_hd) * vhc_pos[1];

    double phi = atan2((obs_pos[obs_idx][1] - vhc_pos[1]), (obs_pos[obs_idx][0] - vhc_pos[0]));
    if ((obs_pos[obs_idx][1] - vhc_pos[1]) < 0) {
        phi = 2 * PI + phi;
    }
    else if ((obs_pos[obs_idx][0] - vhc_pos[0]) == 0 && (obs_pos[obs_idx][1] - vhc_pos[1]) > 0) {
        phi = PI / 2;
    }
    else if ((obs_pos[obs_idx][0] - vhc_pos[0]) == 0 && (obs_pos[obs_idx][1] - vhc_pos[1]) < 0) {
        phi = 3 * PI / 2;
    }

    double ext_r;
    if (fabs(vhc_hd) < fabs(phi)) {
        // Right turn initial conditions check
        double a, b;
        if (fabs(tan(vhc_hd)) < tan(PI / 4)) {
            a = vhc_pos[0];
            b = (vhc_pos[0] < obs_pos[obs_idx][0]) ? obs_pos[obs_idx][0] + r_o : obs_pos[obs_idx][0] - r_o;
        }
        else {
            a = vhc_pos[1];
            b = (vhc_pos[1] < obs_pos[obs_idx][1]) ? obs_pos[obs_idx][1] + r_o : obs_pos[obs_idx][1] - r_o;
        }
        bool err_r = false;
        double f_a = func(a, obs_pos[obs_idx], vhc_hd, c1, c2, del_x_r, del_y_r, beta, R, r_v, r_o, vhc_hd_r, true);
        double f_b = func(b, obs_pos[obs_idx], vhc_hd, c1, c2, del_x_r, del_y_r, beta, R, r_v, r_o, vhc_hd_r, true);

        if (f_a * f_b > 0) {
            err_r = true;
        }

        if (err_r != true) {
            // Initialization
            double t_0_r = (a + b) / 2.0;

            // Iteration
            while (fabs(func(t_0_r, obs_pos[obs_idx], vhc_hd, c1, c2, del_x_r, del_y_r, beta, R, r_v, r_o, vhc_hd_r, 1)) > pow(10, -6)) {
                if (func(t_0_r, obs_pos[obs_idx], vhc_hd, c1, c2, del_x_r, del_y_r, beta, R, r_v, r_o, vhc_hd_r, 1)
                    * func(a, obs_pos[obs_idx], vhc_hd, c1, c2, del_x_r, del_y_r, beta, R, r_v, r_o, vhc_hd_r, 1) > 0) {
                    a = t_0_r;
                }
                else {
                    b = t_0_r;
                }
                t_0_r = (a + b) / 2.0;
            }

            if (fabs(tan(vhc_hd)) < tan(PI / 4)) {
                double tur_pos[2] = { t_0_r, tan(vhc_hd) * t_0_r + c1 };
                ext_r = sqrt(pow((obs_pos[obs_idx][0] - tur_pos[0]), 2) + pow((obs_pos[obs_idx][1] - tur_pos[1]), 2));
            }
            else {
                double tur_pos[2] = {  t_0_r / tan(vhc_hd) + c2, t_0_r };
                ext_r = sqrt(pow((obs_pos[obs_idx][0] - tur_pos[0]), 2) + pow((obs_pos[obs_idx][1] - tur_pos[1]), 2));
            }
        }
        else {
            // Edge case or unable to turn
            double a, b;
            if (fabs(tan(vhc_hd)) < tan(PI / 4)) {
                a = vhc_pos[0] - back_dist * cos(vhc_hd);
                b = (vhc_pos[0] < obs_pos[obs_idx][0]) ? obs_pos[obs_idx][0] + r_o : obs_pos[obs_idx][0] - r_o;
            }
            else {
                a = vhc_pos[1] - back_dist * sin(vhc_hd);
                b = (vhc_pos[1] < obs_pos[obs_idx][1]) ? obs_pos[obs_idx][1] + r_o : obs_pos[obs_idx][1] - r_o;
            }

            double f_a = func(a, obs_pos[obs_idx], vhc_hd, c1, c2, del_x_r, del_y_r, beta, R, r_v, r_o, vhc_hd_r, 1);
            double f_b = func(b, obs_pos[obs_idx], vhc_hd, c1, c2, del_x_r, del_y_r, beta, R, r_v, r_o, vhc_hd_r, 1);

            if (f_a * f_b > 0) {
                ext_r = -1;
            }
            else {
                ext_r = -2;
            }
        }
    }
    else {
        // Left turn initial conditions check
        double a, b;
        if (fabs(tan(vhc_hd)) < tan(PI / 4)) {
            a = vhc_pos[0];
            b = (vhc_pos[0] < obs_pos[obs_idx][0]) ? obs_pos[obs_idx][0] + r_o : obs_pos[obs_idx][0] - r_o;
        }
        else {
            a = vhc_pos[1];
            b = (vhc_pos[1] < obs_pos[obs_idx][1]) ? obs_pos[obs_idx][1] + r_o : obs_pos[obs_idx][1] - r_o;
        }
        int err_l = 0;
        double f_a = func(a, obs_pos[obs_idx], vhc_hd, c1, c2, del_x_l, del_y_l, beta, R, r_v, r_o, vhc_hd_l, 0);
        double f_b = func(b, obs_pos[obs_idx], vhc_hd, c1, c2, del_x_l, del_y_l, beta, R, r_v, r_o, vhc_hd_l, 0);

        if (f_a * f_b > 0) {
            err_l = 1;
        }

        if (err_l != 1) {
            // Initialization
            double t_0_l = (a + b) / 2.0;

            // Iteration
            while (fabs(func(t_0_l, obs_pos[obs_idx], vhc_hd, c1, c2, del_x_l, del_y_l, beta, R, r_v, r_o, vhc_hd_l, 0)) > 1e-6) {
                if (func(t_0_l, obs_pos[obs_idx], vhc_hd, c1, c2, del_x_l, del_y_l, beta, R, r_v, r_o, vhc_hd_l, 0)
                    * func(a, obs_pos[obs_idx], vhc_hd, c1, c2, del_x_l, del_y_l, beta, R, r_v, r_o, vhc_hd_l, 0) > 0) {
                    a = t_0_l;
                }
                else {
                    b = t_0_l;
                }
                t_0_l = (a + b) / 2.0;
            }

            if (fabs(tan(vhc_hd)) < tan(PI / 4)) {
                double tur_pos[2] = { t_0_l, tan(vhc_hd) * t_0_l + c1 };
                ext_r = sqrt(pow((obs_pos[obs_idx][0] - tur_pos[0]), 2) + pow((obs_pos[obs_idx][1] - tur_pos[1]), 2));
            }
            else {
                double tur_pos[2] = {  t_0_l / tan(vhc_hd) + c2, t_0_l };
                ext_r = sqrt(pow((obs_pos[obs_idx][0] - tur_pos[0]), 2) + pow((obs_pos[obs_idx][1] - tur_pos[1]), 2));
            }
        }
        else {
            // Edge case or unable to turn
            double a, b;
            if (fabs(tan(vhc_hd)) < tan(PI / 4)) {
                a = vhc_pos[0] - back_dist * cos(vhc_hd);
                b = (vhc_pos[0] < obs_pos[obs_idx][0]) ? obs_pos[obs_idx][0] + r_o : obs_pos[obs_idx][0] - r_o;
            }
            else {
                a = vhc_pos[1] - back_dist * sin(vhc_hd);
                b = (vhc_pos[1] < obs_pos[obs_idx][1]) ? obs_pos[obs_idx][1] + r_o : obs_pos[obs_idx][1] - r_o;
            }

            double f_a = func(a, obs_pos[obs_idx], vhc_hd, c1, c2, del_x_l, del_y_l, beta, R, r_v, r_o, vhc_hd_l, 0);
            double f_b = func(b, obs_pos[obs_idx], vhc_hd, c1, c2, del_x_l, del_y_l, beta, R, r_v, r_o, vhc_hd_l, 0);

            if (f_a * f_b > 0) {
                ext_r = -1;
            }
            else {
                ext_r = -2;
            }
        }
    }

    return ext_r;
}


int** modi_astar(int startX, int startY, bool** map, int* goal, int connectingDistance, int** obs_pos, double* obs_sz, int obs_num, double* turning_coeff, int height, int width, int* path_len, cv::VideoCapture cap, cv::Mat image)
{
    double** GScore = (double**)malloc(height * sizeof(double*));
    for (int i = 0; i < height; i++)
        GScore[i] = (double*)malloc(width * sizeof(double));

    double** FScore = (double**)malloc(height * sizeof(double*));
    for (int i = 0; i < height; i++)
        FScore[i] = (double*)malloc(width * sizeof(double));

    double** Hn = (double**)malloc(height * sizeof(double*));
    for (int i = 0; i < height; i++)
        Hn[i] = (double*)malloc(width * sizeof(double));

    bool** OpenMAT = (bool**)malloc(height * sizeof(bool*));
    for (int i = 0; i < height; i++)
        OpenMAT[i] = (bool*)malloc(width * sizeof(bool));

    bool** ClosedMAT = (bool**)malloc(height * sizeof(bool*));
    for (int i = 0; i < height; i++)
        ClosedMAT[i] = (bool*)malloc(width * sizeof(bool));
    ClosedMAT = map;

    int** ParentX = (int**)malloc(height * sizeof(int*));
    for (int i = 0; i < height; i++)
        ParentX[i] = (int*)malloc(width * sizeof(int));

    int** ParentY = (int**)malloc(height * sizeof(int*));
    for (int i = 0; i < height; i++)
        ParentY[i] = (int*)malloc(width * sizeof(int));

    //initialize all map
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            GScore[i][j] = 0;
            FScore[i][j] = INF;
            Hn[i][j] = 0;
            OpenMAT[i][j] = 0;
            ParentX[i][j] = 0;
            ParentY[i][j] = 0;
        }
    }

    int size = 2 * connectingDistance + 1;
    int Dummy = 2 * connectingDistance + 1;
    int Mid = connectingDistance;
    bool** NeighboorCheck = (bool**)malloc(size * sizeof(bool*));
    for (int i = 0; i < size; i++)
        NeighboorCheck[i] = (bool*)malloc(size * sizeof(bool));

    for (int i = 0; i < size; i++)
    {
        for (int j = 0; j < size; j++)
        {
            NeighboorCheck[i][j] = 1;
        }
    }
    
    for (int i = 0; i < connectingDistance-1; i++) {
        NeighboorCheck[i][i] = 0;
        NeighboorCheck[Dummy - i - 1][i] = 0;
        NeighboorCheck[i][Dummy - i - 1] = 0;
        NeighboorCheck[Dummy - i - 1][Dummy - i - 1] = 0;
        NeighboorCheck[Mid][i] = 0;
        NeighboorCheck[Mid][Dummy - i - 1] = 0;
        NeighboorCheck[i][Mid] = 0;
        NeighboorCheck[Dummy - i - 1][Mid] = 0;
    }
    NeighboorCheck[Mid][Mid] = 0;

    // Find the indices of elements equal to 1
    int* rows = (int*)malloc((size * size) * sizeof(int));
    int* cols = (int*)malloc((size * size) * sizeof(int));
    int count = 0;

    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            if (NeighboorCheck[i][j] == 1) {
                rows[count] = i;
                cols[count] = j;
                count++;
            }
        }
    }

    // Adjust indices based on Connecting_Distance
    int** Neighboors = (int**)malloc(count * sizeof(int*));
    for (int i = 0; i < count; i++) {
        Neighboors[i] = (int*)malloc(2 * sizeof(int));
        Neighboors[i][1] = rows[i] - connectingDistance;
        Neighboors[i][0] = cols[i] - connectingDistance;
    }

    int N_Neighboors = count;
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            Hn[i][j] = sqrt(pow((j - goal[0]), 2) + pow((i - goal[1]), 2));
        }
    }

    FScore[startY][startX] = Hn[startY][startX];
    OpenMAT[startY][startX] = 1;
    ParentX[startY][startX] = 0;
    ParentY[startY][startX] = 0;

    bool RECONSTRUCTPATH;
    double obs_adj_r[3] = { 0. };
    double vhc_pos[2] = { 0 };
    double vhc_hd = 0.0;
    int CurrentY, CurrentX;
    int cnt = 0;
    while (1) 
    {
        cnt++;
        if (cnt % 1000 == 0)
            cap.read(image);

        double MINopenFSCORE = INF;
        CurrentY = -1;
        CurrentX = -1;
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                if (FScore[i][j] < MINopenFSCORE) {
                    MINopenFSCORE = FScore[i][j];
                    CurrentY = i;
                    CurrentX = j;
                }
            }
        }
//printf("%d %d\n", CurrentX, CurrentY);
        // Check if the minimum value is infinity
        if (MINopenFSCORE == INF)       // Failure!
        {
            RECONSTRUCTPATH = 0;
            printf("Failure! Couldnt find the optimal path\n");
            return 0;
        }

        if (CurrentY == goal[1] && CurrentX == goal[0])     //arrive!!
        {
            RECONSTRUCTPATH = 1;
            break;
        }    

        OpenMAT[CurrentY][CurrentX] = 0;
        FScore[CurrentY][CurrentX] = INF;
        ClosedMAT[CurrentY][CurrentX] = 1;

        // Calculate vhc_hd using atan2
        double vhc_hd = atan2((double)(CurrentY - ParentY[CurrentY][CurrentX]), (double)(CurrentX - ParentX[CurrentY][CurrentX]));

        // Adjust vhc_hd based on the quadrant
        if ((CurrentY - ParentY[CurrentY][CurrentX]) < 0)       // Third and fourth quadrants
            vhc_hd = 2 * PI + vhc_hd;
        else if ((CurrentX - ParentX[CurrentY][CurrentX]) == 0 && (CurrentY - ParentY[CurrentY][CurrentX]) > 0)
            vhc_hd = PI / 2;
        else if ((CurrentX - ParentX[CurrentY][CurrentX]) == 0 && (CurrentY - ParentY[CurrentY][CurrentX]) < 0)
            vhc_hd = 3 * PI / 2;

        vhc_pos[0] = CurrentX; vhc_pos[1] = CurrentY;

        // Loop through each obstacle
        for (int obs_idx = 0; obs_idx < obs_num; obs_idx++) {
            double dist = sqrt(pow((CurrentY - obs_pos[obs_idx][1]), 2) + pow((CurrentX - obs_pos[obs_idx][0]), 2));
            if (dist < obs_sz[obs_idx] + 2 * turning_coeff[5] + turning_coeff[0]) {  // 2R + r_v
                if (CurrentX == startX && CurrentY == startY) {
                    vhc_hd = atan2((CurrentY - obs_pos[obs_idx][1]), (CurrentX - obs_pos[obs_idx][0]));
                    if ((CurrentY - obs_pos[obs_idx][1]) < 0) { // Third and fourth quadrants
                        vhc_hd = 2 * PI + vhc_hd;
                    }
                    else if ((CurrentX - obs_pos[obs_idx][0]) == 0 && (CurrentY - obs_pos[obs_idx][1]) > 0) {
                        vhc_hd = PI / 2;
                    }
                    else if ((CurrentX - obs_pos[obs_idx][0]) == 0 && (CurrentY - obs_pos[obs_idx][1]) < 0) {
                        vhc_hd = 3 * PI / 2;
                    }
                }
//printf("%d %d %d %lf\n", CurrentX, CurrentY, obs_idx, vhc_hd);
                obs_adj_r[obs_idx] = obs_ext_radi(&(vhc_pos[0]), vhc_hd, obs_idx, obs_pos, obs_sz, turning_coeff);
//printf("%d %d %d %lf\n", CurrentX, CurrentY, obs_idx, obs_adj_r[obs_idx]);

                if (obs_adj_r[obs_idx] == -1) {
                    obs_adj_r[obs_idx] = obs_sz[obs_idx] + turning_coeff[0];
                }
                else if (obs_adj_r[obs_idx] == -2) {
                    if (dist > obs_sz[obs_idx] + turning_coeff[0]) {
                        obs_adj_r[obs_idx] = dist;
                    }
                    else {
                        obs_adj_r[obs_idx] = obs_sz[obs_idx] + turning_coeff[0];
                    }
                }
            }
            else {
                obs_adj_r[obs_idx] = obs_sz[obs_idx] + turning_coeff[0];
            }
        }
//for(int obs_idx = 0; obs_idx < obs_num; obs_idx++)
//    printf("%d %d %lf\n", CurrentX, CurrentY, obs_adj_r[obs_idx]);
//for (int p = 0; p < N_Neighboors; p++)
//    printf("%d %d\n", Neighboors[p][0], Neighboors[p][1]);
//return 0;
        for (int p = 0; p < N_Neighboors; p++) {
            int i = round(Neighboors[p][0] * 0.8);  // Y
            int j = round(Neighboors[p][1] * 0.8
            );  // X

            if (CurrentY + i < 0 || CurrentY + i >= height || CurrentX + j < 0 || CurrentX + j >= width) {
                continue;
            }
//printf("%d %d %d %d %d\n", CurrentX, CurrentY, j, i, ClosedMAT[CurrentY + i][CurrentX + j]);
            if (ClosedMAT[CurrentY + i][CurrentX + j] == 0) {  // Neighbor is open
                // Need to check that the path does not pass an object
                int Flag = 1;
                for (int obs_idx = 0; obs_idx < obs_num; obs_idx++) {
                    double ax = (double)CurrentX - obs_pos[obs_idx][0];
                    double ay = (double)CurrentY - obs_pos[obs_idx][1];
                    double bx = (double)(CurrentX + j) - obs_pos[obs_idx][0];
                    double by = (double)(CurrentY + i) - obs_pos[obs_idx][1];
//printf("%d %d %d %d %lf %lf %lf %lf\n", CurrentX, CurrentY, j, i, ax, ay, bx, by);
                    double a = (bx - ax) * (bx - ax) + (by - ay) * (by - ay);
                    double b = 2 * (ax * (bx - ax) + ay * (by - ay));
                    double c = ax * ax + ay * ay - pow((obs_adj_r[obs_idx] + 2), 2);
                    double det = b * b - 4 * a * c;
//printf("%d %d %d %d %lf %lf\n", CurrentX, CurrentY, j, i, obs_adj_r[obs_idx]);
//printf("%lf\n", det);
                    if (det == 0) {  // Intersects at one point
                        double t1 = (-b + sqrt(det)) / (2 * a);
                        if (0 < t1 && t1 <= 1) {
                            Flag = 0;
                            break;
                        }
                    }
                    else if (det > 0) {
                        double t1 = (-b + sqrt(det)) / (2 * a);
                        double t2 = (-b - sqrt(det)) / (2 * a);
//printf("%d %d %d %d %lf %lf %lf %d\n", CurrentX, CurrentY, j, i, det, t1, t2, Flag);
                        if (t1 > 0 && t2 <= 1) {
                            Flag = 0;

                            break;
                        }
                    }
                }
                    // End of checking that the path does not pass an object
//printf("%d %d %d %d %d\n", CurrentX, CurrentY, j, i, Flag);
//if (j == 1 && i == 0)
//    printf("%d %d %d\n", CurrentX, CurrentY, Flag);
                if (Flag == 1) 
                {
                    double tentative_gScore = GScore[CurrentY][CurrentX] + sqrt(i * i + j * j);
                    if (OpenMAT[CurrentY + i][CurrentX + j] == 0)
                        OpenMAT[CurrentY + i][CurrentX + j] = 1;
                    else if (tentative_gScore >= GScore[CurrentY + i][CurrentX + j])
                        continue;
                    ParentX[CurrentY + i][CurrentX + j] = CurrentX;
                    ParentY[CurrentY + i][CurrentX + j] = CurrentY;
                    GScore[CurrentY + i][CurrentX + j] = tentative_gScore;
                    FScore[CurrentY + i][CurrentX + j] = tentative_gScore + Hn[CurrentY + i][CurrentX + j];
        //printf("%d %d %d %d %lf\n", CurrentX, CurrentY, j, i, FScore[CurrentY + i][CurrentX + j]);
                }
            
            }
        }
    }
    int k = 1;
    //*path_len = 1;
    int end_Y = CurrentY;
    int end_X = CurrentX;
    if (RECONSTRUCTPATH) {
        while (RECONSTRUCTPATH) {
            int CurrentXDummy = ParentX[CurrentY][CurrentX];
            CurrentY = ParentY[CurrentY][CurrentX];
            CurrentX = CurrentXDummy;
            k++;
            if (CurrentX == startX && CurrentY == startY) {
                break;
            }
        }
    }

    *(path_len) = k;

    int** OptimalPath = (int**)malloc(k * sizeof(int*));
    for (int i = 0; i < k; i++)
        OptimalPath[i] = (int*)malloc(2 * sizeof(int));

    CurrentY = end_Y;
    CurrentX = end_X;
    k = 0;
    if (RECONSTRUCTPATH) {
        OptimalPath[k][1] = CurrentY;
        OptimalPath[k][0] = CurrentX;
        while (RECONSTRUCTPATH) {
            int CurrentXDummy = ParentX[CurrentY][CurrentX];
            CurrentY = ParentY[CurrentY][CurrentX];
            CurrentX = CurrentXDummy;
            OptimalPath[k+1][1] = CurrentY;
            OptimalPath[k+1][0] = CurrentX;
            k++;
            if (CurrentX == startX && CurrentY == startY) {
                break;
            }
        }
    }
    return OptimalPath;
}

int** org_astar(int startX, int startY, bool** map, int* goal, int connectingDistance, int** obs_pos, double* obs_sz, int obs_num, double* turning_coeff, int height, int width, int* path_len, cv::VideoCapture cap, cv::Mat image)
{
    double** GScore = (double**)malloc(height * sizeof(double*));
    for (int i = 0; i < height; i++)
        GScore[i] = (double*)malloc(width * sizeof(double));

    double** FScore = (double**)malloc(height * sizeof(double*));
    for (int i = 0; i < height; i++)
        FScore[i] = (double*)malloc(width * sizeof(double));

    double** Hn = (double**)malloc(height * sizeof(double*));
    for (int i = 0; i < height; i++)
        Hn[i] = (double*)malloc(width * sizeof(double));

    bool** OpenMAT = (bool**)malloc(height * sizeof(bool*));
    for (int i = 0; i < height; i++)
        OpenMAT[i] = (bool*)malloc(width * sizeof(bool));

    bool** ClosedMAT = (bool**)malloc(height * sizeof(bool*));
    for (int i = 0; i < height; i++)
        ClosedMAT[i] = (bool*)malloc(width * sizeof(bool));
    ClosedMAT = map;

    int** ParentX = (int**)malloc(height * sizeof(int*));
    for (int i = 0; i < height; i++)
        ParentX[i] = (int*)malloc(width * sizeof(int));

    int** ParentY = (int**)malloc(height * sizeof(int*));
    for (int i = 0; i < height; i++)
        ParentY[i] = (int*)malloc(width * sizeof(int));

    //initialize all map
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            GScore[i][j] = 0;
            FScore[i][j] = INF;
            Hn[i][j] = 0;
            OpenMAT[i][j] = 0;
            ParentX[i][j] = 0;
            ParentY[i][j] = 0;
        }
    }

    int size = 2 * connectingDistance + 1;
    int Dummy = 2 * connectingDistance + 1;
    int Mid = connectingDistance;
    bool** NeighboorCheck = (bool**)malloc(size * sizeof(bool*));
    for (int i = 0; i < size; i++)
        NeighboorCheck[i] = (bool*)malloc(size * sizeof(bool));

    for (int i = 0; i < size; i++)
    {
        for (int j = 0; j < size; j++)
        {
            NeighboorCheck[i][j] = 1;
        }
    }

    for (int i = 0; i < connectingDistance - 1; i++) {
        NeighboorCheck[i][i] = 0;
        NeighboorCheck[Dummy - i - 1][i] = 0;
        NeighboorCheck[i][Dummy - i - 1] = 0;
        NeighboorCheck[Dummy - i - 1][Dummy - i - 1] = 0;
        NeighboorCheck[Mid][i] = 0;
        NeighboorCheck[Mid][Dummy - i - 1] = 0;
        NeighboorCheck[i][Mid] = 0;
        NeighboorCheck[Dummy - i - 1][Mid] = 0;
    }
    NeighboorCheck[Mid][Mid] = 0;

    // Find the indices of elements equal to 1
    int* rows = (int*)malloc((size * size) * sizeof(int));
    int* cols = (int*)malloc((size * size) * sizeof(int));
    int count = 0;

    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            if (NeighboorCheck[i][j] == 1) {
                rows[count] = i;
                cols[count] = j;
                count++;
            }
        }
    }

    // Adjust indices based on Connecting_Distance
    int** Neighboors = (int**)malloc(count * sizeof(int*));
    for (int i = 0; i < count; i++) {
        Neighboors[i] = (int*)malloc(2 * sizeof(int));
        Neighboors[i][1] = rows[i] - connectingDistance;
        Neighboors[i][0] = cols[i] - connectingDistance;
    }

    int N_Neighboors = count;
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            Hn[i][j] = sqrt(pow((j - goal[0]), 2) + pow((i - goal[1]), 2));
        }
    }

    FScore[startY][startX] = Hn[startY][startX];
    OpenMAT[startY][startX] = 1;
    ParentX[startY][startX] = 0;
    ParentY[startY][startX] = 0;

    bool RECONSTRUCTPATH;
    int CurrentY, CurrentX;
    int cnt = 0;
    while (1)
    {
        cnt++;
        if (cnt % 2000 == 0)
            cap.read(image);

        double MINopenFSCORE = INF;
        CurrentY = -1;
        CurrentX = -1;
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                if (FScore[i][j] < MINopenFSCORE) {
                    MINopenFSCORE = FScore[i][j];
                    CurrentY = i;
                    CurrentX = j;
                }
            }
        }
        //printf("%d %d\n", CurrentX, CurrentY);
                // Check if the minimum value is infinity
        if (MINopenFSCORE == INF)       // Failure!
        {
            RECONSTRUCTPATH = 0;
            printf("Failure! Couldnt find the optimal path\n");
            return 0;
        }

        if (CurrentY == goal[1] && CurrentX == goal[0])     //arrive!!
        {
            RECONSTRUCTPATH = 1;
            break;
        }

        OpenMAT[CurrentY][CurrentX] = 0;
        FScore[CurrentY][CurrentX] = INF;
        ClosedMAT[CurrentY][CurrentX] = 1;

        for (int p = 0; p < N_Neighboors; p++) {
            int i = round(Neighboors[p][0] * 0.8);  // Y
            int j = round(Neighboors[p][1] * 0.8);  // X

            if (CurrentY + i < 0 || CurrentY + i >= height || CurrentX + j < 0 || CurrentX + j >= width) {
                continue;
            }
            //printf("%d %d %d %d %d\n", CurrentX, CurrentY, j, i, ClosedMAT[CurrentY + i][CurrentX + j]);
            if (ClosedMAT[CurrentY + i][CurrentX + j] == 0) {  // Neighbor is open
                // Need to check that the path does not pass an object
                int Flag = 1;
                for (int obs_idx = 0; obs_idx < obs_num; obs_idx++) {
                    double ax = (double)CurrentX - obs_pos[obs_idx][0];
                    double ay = (double)CurrentY - obs_pos[obs_idx][1];
                    double bx = (double)(CurrentX + j) - obs_pos[obs_idx][0];
                    double by = (double)(CurrentY + i) - obs_pos[obs_idx][1];
                    //printf("%d %d %d %d %lf %lf %lf %lf\n", CurrentX, CurrentY, j, i, ax, ay, bx, by);
                    double a = (bx - ax) * (bx - ax) + (by - ay) * (by - ay);
                    double b = 2 * (ax * (bx - ax) + ay * (by - ay));
                    double c = ax * ax + ay * ay - pow(((obs_sz[obs_idx]) + 2), 2);
                    double det = b * b - 4 * a * c;
                    //printf("%d %d %d %d %lf %lf\n", CurrentX, CurrentY, j, i, obs_adj_r[obs_idx]);
                    //printf("%lf\n", det);
                    if (det == 0) {  // Intersects at one point
                        double t1 = (-b + sqrt(det)) / (2 * a);
                        if (0 < t1 && t1 <= 1) {
                            Flag = 0;
                            break;
                        }
                    }
                    else if (det > 0) {
                        double t1 = (-b + sqrt(det)) / (2 * a);
                        double t2 = (-b - sqrt(det)) / (2 * a);
                        //printf("%d %d %d %d %lf %lf %lf %d\n", CurrentX, CurrentY, j, i, det, t1, t2, Flag);
                        if (t1 > 0 && t2 <= 1) {
                            Flag = 0;

                            break;
                        }
                    }
                }
                // End of checking that the path does not pass an object
//printf("%d %d %d %d %d\n", CurrentX, CurrentY, j, i, Flag);
//if (j == 1 && i == 0)
//    printf("%d %d %d\n", CurrentX, CurrentY, Flag);
                if (Flag == 1)
                {
                    double tentative_gScore = GScore[CurrentY][CurrentX] + sqrt(i * i + j * j);
                    if (OpenMAT[CurrentY + i][CurrentX + j] == 0)
                        OpenMAT[CurrentY + i][CurrentX + j] = 1;
                    else if (tentative_gScore >= GScore[CurrentY + i][CurrentX + j])
                        continue;
                    ParentX[CurrentY + i][CurrentX + j] = CurrentX;
                    ParentY[CurrentY + i][CurrentX + j] = CurrentY;
                    GScore[CurrentY + i][CurrentX + j] = tentative_gScore;
                    FScore[CurrentY + i][CurrentX + j] = tentative_gScore + Hn[CurrentY + i][CurrentX + j];
                    //printf("%d %d %d %d %lf\n", CurrentX, CurrentY, j, i, FScore[CurrentY + i][CurrentX + j]);
                }

            }
        }
    }
    int k = 1;
    //*path_len = 1;
    int end_Y = CurrentY;
    int end_X = CurrentX;
    if (RECONSTRUCTPATH) {
        while (RECONSTRUCTPATH) {
            int CurrentXDummy = ParentX[CurrentY][CurrentX];
            CurrentY = ParentY[CurrentY][CurrentX];
            CurrentX = CurrentXDummy;
            k++;
            if (CurrentX == startX && CurrentY == startY) {
                break;
            }
        }
    }

    *(path_len) = k;

    int** OptimalPath = (int**)malloc(k * sizeof(int*));
    for (int i = 0; i < k; i++)
        OptimalPath[i] = (int*)malloc(2 * sizeof(int));

    CurrentY = end_Y;
    CurrentX = end_X;
    k = 0;
    if (RECONSTRUCTPATH) {
        OptimalPath[k][1] = CurrentY;
        OptimalPath[k][0] = CurrentX;
        while (RECONSTRUCTPATH) {
            int CurrentXDummy = ParentX[CurrentY][CurrentX];
            CurrentY = ParentY[CurrentY][CurrentX];
            CurrentX = CurrentXDummy;
            OptimalPath[k + 1][1] = CurrentY;
            OptimalPath[k + 1][0] = CurrentX;
            k++;
            if (CurrentX == startX && CurrentY == startY) {
                break;
            }
        }
    }
    return OptimalPath;
}

double** b_spline(int** ini_path, int size)
{
    double* X = (double*)malloc((size + 4) * sizeof(double));
    double* Y = (double*)malloc((size + 4) * sizeof(double));

    double** C = (double**)malloc(((size + 1) * 4) * sizeof(double*));
    for (int i = 0; i < ((size + 1) * 4); i++)
    {
        C[i] = (double*)malloc(2 * sizeof(double));
        C[i][0] = 0;          //initailize
        C[i][1] = 0;          //initailize
    }

    for (int i = 0; i < size; i++) {
        X[i + 2] = (double)ini_path[i][0];
        Y[i + 2] = (double)ini_path[i][1];
    }
    X[0] = X[1] = (double)ini_path[0][0];
    X[size + 2] = X[size + 3] = (double)ini_path[size - 1][0];
    Y[0] = Y[1] = (double)ini_path[0][1];
    Y[size + 2] = Y[size + 3] = (double)ini_path[size - 1][1];
        
    double B[4][4] = {
        {1, 4, 1, 0},
        {-3, 0, 3, 0},
        {3, -6, 3, 0},
        {-1, 3, -3, 1}
    };

    double A[4][4];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            A[i][j] = (1.0 / 6) * B[i][j];
        }
    }

    double T[4] = { 0.0 };
    double coff[4] = { 0.0 };
    T[0] = 1.;
    for (int i = 1; i < size + 2; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            T[1] = pow(0.3 * (double)j, 1);
            T[2] = pow(0.3 * (double)j, 2);
            T[3] = pow(0.3 * (double)j, 3);

            for (int k = 0; k < 4; k++)
            {
                coff[k] = 0;
                for (int m = 0; m < 4; m++)
                {
                    // printf("%lf %lf %d\n", T[m], A[m][k], j);
                    coff[k] += T[m] * A[m][k];
                }
            }

            // for (int m = 0; m < 4; m++)
            //     printf("%lf %d\n", coff[m], j);

            for (int k = 0; k < 4; k++)
            {
                C[(i - 1) * 4 + j][0] += coff[k] * X[i - 1 + k];
                C[(i - 1) * 4 + j][1] += coff[k] * Y[i - 1 + k];
            }
            //printf("%lf %lf\n", C[(i - 1) * 4 + j][0], C[(i - 1) * 4 + j][1]);
        }
    }
    //for (int i = 0; i < (size + 1) * 4; i++)
    //    printf("%lf %lf %d\n", C[i][0], C[i][1], i);
    free(X);
    free(Y);
    return C;
}

double angletest(double* preStartPose, double* startPose, double* goalPose) {
    double vector1[2] = { startPose[1] - preStartPose[1], startPose[0] - preStartPose[0] };
    double vector2[2] = { goalPose[1] - startPose[1], goalPose[0] - startPose[0] };

    double dot_product = vector1[0] * vector2[0] + vector1[1] * vector2[1];
    double determinant = vector2[0] * vector1[1] - vector2[1] * vector1[0];
    double angle = atan2(determinant, dot_product);
    if (dot_product == 0 && determinant > 0) {
        angle = PI / 2;
    }
    else if (dot_product == 0 && determinant < 0) {
        angle = -PI / 2;
    }

    //if (angle > PI) {
    //    angle -= 2 * PI;
    //}
    //else if (angle < -PI) {
    //    angle += 2 * PI;
    //}

    return -angle;      //掛負號是因為實驗室地圖y軸向下為正
}

void disTri(double x, double a, double b, double c, double* newX, int* mf) {
    if (x >= a && x <= b) {
        *newX = (x - a) / (b - a);
    }
    else if (x > b && x <= c) {
        *newX = (c - x) / (c - b);
    }
    else {
        *newX = 0.0;
    }
    if (b == 6) {
        *mf = PS;
    }
    else if (b == 9) {
        *mf = PM;
    }
    else if (b == 12) {
        *mf = PB;
    }
}

void disTrapR(double x, double a, double b, double* newX, int* mf) {
    if (x >= a && x <= b) {
        *newX = (x - a) / (b - a);
    }
    else if (x > b) {
        *newX = 1.0;
    }
    else {
        *newX = 0.0;
    }
    *mf = PVB; // 4
}

void disTrapL(double x, double a, double b, double* newX, int* mf) {
    if (x >= a && x <= b) {
        *newX = (b - x) / (b - a);
    }
    else if (x < a) {
        *newX = 1.0;
    }
    else {
        *newX = 0.0;
    }
    *mf = ZO; // 0
}

void angTri(double x, double a, double b, double c, double* newX, int* mf) {
    if (x >= a && x <= b) {
        *newX = (x - a) / (b - a);
    }
    else if (x > b && x <= c) {
        *newX = (c - x) / (c - b);
    }
    else {
        *newX = 0.0;
    }
    if (b == -PI / 4) {
        *mf = NS;
    }
    else if (b == 0) {
        *mf = ZO;
    }
    else if (b == PI / 4) {
        *mf = PS;
    }
}

void angTrapR(double x, double a, double b, double* newX, int* mf) {
    if (x >= a && x <= b) {
        *newX = (x - a) / (b - a);
    }
    else if (x > b) {
        *newX = 1.0;
    }
    else {
        *newX = 0.0;
    }
    *mf = PB; // PB
}

void angTrapL(double x, double a, double b, double* newX, int* mf) {
    if (x >= a && x <= b) {
        *newX = (b - x) / (b - a);
    }
    else if (x < a) {
        *newX = 1.0;
    }
    else {
        *newX = 0.0;
    }
    *mf = NB; // NB
}

void rule(int drMf, int arMf, int* RMF, int* LMF, int* time) {
    if (drMf == ZO) {
        if (arMf == NB) {
            *LMF = PS;
            *RMF = ZO;
            *time = PS;
        }
        else if (arMf == NS) {
            *LMF = PS;
            *RMF = ZO;
            *time = PM;
        }
        else if (arMf == ZO) {
            *LMF = PS;
            *RMF = PS;
            *time = PM;
        }
        else if (arMf == PS) {
            *LMF = ZO;
            *RMF = PS;
            *time = PM;
        }
        else if (arMf == PB) {
            *LMF = ZO;
            *RMF = PS;
            *time = PS;
        }
    }
    else if (drMf == PS) {
        if (arMf == NB) {
            *LMF = PM;
            *RMF = ZO;
            *time = PS;
        }
        else if (arMf == NS) {
            *LMF = PM;
            *RMF = PS;
            *time = PS;
        }
        else if (arMf == ZO) {
            *LMF = PM;
            *RMF = PM;
            *time = PM;
        }
        else if (arMf == PS) {
            *LMF = PS;
            *RMF = PM;
            *time = PS;
        }
        else if (arMf == PB) {
            *LMF = ZO;
            *RMF = PM;
            *time = PS;
        }
    }
    else if (drMf == PM) {
        if (arMf == NB) {
            *LMF = PB;
            *RMF = ZO;
            *time = PS;
        }
        else if (arMf == NS) {
            *LMF = PB;
            *RMF = PS;
            *time = PS;
        }
        else if (arMf == ZO) {
            *LMF = PB;
            *RMF = PB;
            *time = PS;
        }
        else if (arMf == PS) {
            *LMF = PS;
            *RMF = PB;
            *time = PS;
        }
        else if (arMf == PB) {
            *LMF = ZO;
            *RMF = PB;
            *time = PS;
        }
    }
    else if (drMf == PB) {
        if (arMf == NB) {
            *LMF = PVB;
            *RMF = PS;
            *time = PM;
        }
        else if (arMf == NS) {
            *LMF = PVB;
            *RMF = PM;
            *time = ZO;
        }
        else if (arMf == ZO) {
            *LMF = PVB;
            *RMF = PVB;
            *time = ZO;
        }
        else if (arMf == PS) {
            *LMF = PM;
            *RMF = PVB;
            *time = ZO;
        }
        else if (arMf == PB) {
            *LMF = PS;
            *RMF = PVB;
            *time = PM;
        }
    }
    else if (drMf == PVB) {
        if (arMf == NB) {
            *LMF = PVB;
            *RMF = PM;
            *time = PM;
        }
        else if (arMf == NS) {
            *LMF = PVB;
            *RMF = PB;
            *time = ZO;
        }
        else if (arMf == ZO) {
            *LMF = PVB;
            *RMF = PVB;
            *time = ZO;
        }
        else if (arMf == PS) {
            *LMF = PB;
            *RMF = PVB;
            *time = ZO;
        }
        else if (arMf == PB) {
            *LMF = PM;
            *RMF = PVB;
            *time = PM;
        }
    }
}

void defuzzy(double* accR, double* accL, double* accEpoch, double yweigh, int Rmf, int Lmf, int time) {
    *accL += (double)Lmf * yweigh * 0.5;
    *accR += (double)Rmf * yweigh * 0.5;
    *accEpoch += (double)time * yweigh * 0.5;
}

double steering_val(double psi)
{
    double str_control;
    if (psi < 0)
        str_control = -3.3101 * psi - 5.8605;
    else
        str_control = -2.2468 * psi - 9.3995;

    return str_control;
}

bool collision_check(int** obs_pos, double* obs_sz, double* vhc_pos, double r_v, int obs_num)
{
    for (int i = 0; i < obs_num; i++)
    {
        double dist = sqrt(pow((vhc_pos[1] - obs_pos[i][1]), 2) + pow((vhc_pos[0] - obs_pos[i][0]), 2));
        if (dist < (r_v + obs_sz[i]))
            return true;
    }
    return false;
}

bool waitCarReady() {
    char Version[16];
    while (1) {
        int state = myRacer.getVersion(Version);
        if (state == 0) {
            printf("F/W VERSION = %s\r\n", Version);
            break;
        }
        printf("NOT READY %d. Keep waiting or press 'q' to QUIT\r\n", state);
        //printf("4 5 6 to select IP\r\n");
        for (int i = 0; i < 10; i++) {
            if (_kbhit()) {
                unsigned char ch = _getch();
                if ((ch == 'q') || (ch == 'Q') || (ch == ' ')) {
                    myRacer.close();
                    return false;
                }
            }
            Sleep(100);
        }
    }
    ////myRacer.buzzerOn(1);
    ////Sleep(50);
    ////myRacer.buzzerOn(0);
    //printf("press the following keys to run\r\n");
    ////printf("  SELECT CAR : 4 5 6\r\n");
    ////printf("  LED : r g o\r\n");
    //printf("  MOTOR : w a s d x W A S D X\r\n");
    ////printf("  Speed : e \r\n");
    //printf("  Rplidar : l L\r\n");
    ////printf("  BUZZER : b\r\n");
    //printf("  9AXIS : 9\r\n");
    //printf("  QUIT : q Q SPACE\r\n");
    return true;
}


int main(int argc, char** argv) {
	//car settinng
//int Angle[3], AngularVelocity[3], Acceleration[3], Magnetic[3];
	char CAR_IP[64] = DEF_CAR_IP;

	//chose car
	if (argc > 1) {
		strcpy_s(CAR_IP, argv[1]);
		if (CAR_IP[0] == 0) {
			strcpy_s(CAR_IP, DEF_CAR_IP);
		}
	}

	if (myRacer.init() < 0) {
		printf("INIT ERR\r\n");
	}
	else
	{
		myRacer.debug = true;
		printf("CAR IP = %s\r\n", CAR_IP);
		myRacer.select(CAR_IP);
		if (!waitCarReady()) {
			return 0;
		}
	}

	//openCV攝影機環境設定
	std::cout << "Webcam\n" << std::endl;
	cv::Point2d center, heading;
	std::string StreamAddress = "http://root:hellokitty@10.0.0.100/axis-cgi/mjpg/video.cgi?resolution=640x480&req_fps=30&.mjpg";
	std::string window_name = "Webcam";
	cv::VideoCapture cap(StreamAddress, CV_CAP_FFMPEG);

	// Read undistortion data
	std::string instrinc_name = "o6_intrinsic.xml"; // Camera matrix
	cv::FileStorage fs(instrinc_name, cv::FileStorage::READ);
	cv::Mat intrinsic_matrix_loaded, distortion_coeffs_loaded;
	cv::Size image_size;
	fs["camera_matrix"] >> intrinsic_matrix_loaded;
	fs["distortion_coefficients"] >> distortion_coeffs_loaded;
	fs["image_width"] >> image_size.width;
	fs["image_height"] >> image_size.height;
	// 影像校正要用
	cv::Mat image, frame, map1, map2; // Webcam Calibration
	cv::Mat subFrame, dst, bigDst, markedCanny; // Object Detection
	std::vector<cv::Vec4i> lines; // Heading
	cv::initUndistortRectifyMap(intrinsic_matrix_loaded, distortion_coeffs_loaded,
		cv::Mat(), intrinsic_matrix_loaded, image_size, CV_16SC2, map1, map2);

	// Show fps
	std::clock_t time;
	short fps = 0;

	//int key;

	//紀錄軌跡 :279行設定完成
	//int frameWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH);
	//int frameHeigth = cap.get(cv::CAP_PROP_FRAME_HEIGHT);

	//cv::Mat tracking_frame(frameHeigth, frameWidth, CV_64FC4, cv::Scalar(255, 255, 255));

	// Create video writer object
	//cv::VideoWriter output("output.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30, cv::Size(frameWidth, frameHeigth));

	// Create tracker, select region-of-interest (ROI) and initialize the tracker
	cv::Ptr<cv::MultiTracker> multiTracker = cv::MultiTracker::create();

	cap.read(image);
	cv::remap(image, frame, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
	






	//載具設定
	double vhc[2] = {0.0};
	//double goal[2] = { boundingBoxes[1].x + boundingBoxes[1].width / 2.,boundingBoxes[1].y + boundingBoxes[1].height / 2. };
	//const double vhc_width = boundingBoxes[0].width;		//載具寬pix
	//const double vhc_length = boundingBoxes[0].height;	//載具長pix
	const double r_v = 40; // 經過影像校正
	double v = 10;				// 載具速度
	double psi = 20;			// 載具最大轉角(degree)
	double psi_rate = 10;		// 載具rate of sheering angle(degree / sec)
	double L_f = 26.7468, L_r = 19.1048;	// 載具前後輪軸距
	double L = L_f + L_r;
	double L_c = 44.4188;
	double rw = 8.9554;	// 輪子半徑




	//double obs_r = 40;
	double org[2] = { 0 ,0 };
	double vhc_hd[2] = { 0 };
	double Distance;
	double angle;
	double Angle;

	// pre - calculation
	double R = sqrt(pow(((double)L_r * tan(abs(psi) * PI / 180)), 2) + pow((double)L, 2)) / tan(abs(psi) * PI / 180);
	double beta = atan((((double)L_r / L) * tan(abs(psi) * PI / 180)));
	psi_rate = psi_rate * PI / 180;

	//障礙物位置
	int obs_num = 3;
	int** obs_pos = (int**)malloc(obs_num * sizeof(int*));
	for (int i = 0; i < obs_num; i++)
		obs_pos[i] = (int*)malloc(2 * sizeof(int));
	double* obs_sz = (double*)malloc(obs_num * sizeof(double));
	//case 1
	//obs_pos[0][0] = 113; obs_pos[1][0] = 106; obs_pos[2][0] = 60;		//x座標
	//obs_pos[0][1] = 92; obs_pos[1][1] = 141; obs_pos[2][1] = 112;		//y座標
	//obs_sz[0] = 17.; obs_sz[1] = 15.; obs_sz[2] = 22.;

 //   obs_pos[0][0] = 420.; obs_pos[1][0] = 370.; obs_pos[2][0] = 210.;		//x座標
 //   obs_pos[0][1] = 150.; obs_pos[1][1] = 340.; obs_pos[2][1] = 200.;		//y座標
 //   obs_sz[0] = 56.; obs_sz[1] = 50.; obs_sz[2] = 73.;

    //case 2
    //obs_pos[0][0] = 112; obs_pos[1][0] = 75; obs_pos[2][0] = 55; /*obs_pos[2][0] = 59;*/	    //x座標
    //obs_pos[0][1] = 90; obs_pos[1][1] = 62; obs_pos[2][1] = 112; /*obs_pos[2][1] = 109;*/		//y座標
    //obs_sz[0] = 22.; obs_sz[1] = 15.; obs_sz[2] = 17.;

    obs_pos[0][0] = 415.; obs_pos[1][0] = 198.; obs_pos[2][0] = 260.;		//x座標
    obs_pos[0][1] = 320.; obs_pos[1][1] = 357.; obs_pos[2][1] = 170.;		//y座標
    //obs_pos[1][0] = 203.; obs_pos[1][1] = 352.;                           //無法通過
    obs_sz[0] = 73.; obs_sz[1] = 56.; obs_sz[2] = 50.;




	//地圖參數
    bool mode = true;			//true -> modified a*; false -> original a*
    int height = image_size.height;
    int width = image_size.width;

	bool** MAP = (bool**)malloc(height * sizeof(bool*));
	for (int i = 0; i < height; i++)
		MAP[i] = (bool*)malloc(width * sizeof(bool));

	for (int i = 0; i < image_size.height; i++)
	{
		for (int j = 0; j < image_size.width; j++)
		{
			MAP[i][j] = 0;
		}
	}

    //case 1
	int StartX = 80;
	int StartY = 80;
    //int goal[2] = { 140, 120 };			//goal -> (x,y) = (140, 120)
    //int goal[2] = { 494, 238 };	
    int goal[2] = { 550, 260 };
    //case 2
    StartX = 80;
    StartY = 80;
    goal[0] = 510; goal[1] = 430;

	int Connecting_Distance = 8;
	double turning_coeff[7] = {0};		//turning_coeff = [r_v psi_rate v L L_r R beta];
	turning_coeff[0] = r_v; turning_coeff[1] = psi_rate; turning_coeff[2] = v;
    turning_coeff[3] = L; turning_coeff[4] = L_r; turning_coeff[5] = R; turning_coeff[6] = beta;         

	//for (int i = 0; i < image_size.height; i++) {
	//	for (int j = 0; j < image_size.width; j++) {
	//		for (int k = 0; k < obs_num; k++) {
	//			if (sqrt(pow(j - obs_pos[k][0], 2) + pow(i - obs_pos[k][1], 2)) < obs_sz[k] + turning_coeff[0] + 2) 
	//				MAP[i][j] = 1;
	//		}
	//	}
	//}

    //-----------------------標定--------------
    //bool key = true;
    cout << "Please place the vehicle at the starting position\n" << endl;

    //center.x = StartX;	center.y = StartY;
    //circle(frame, center, 2, Scalar(0, 255, 0), 1, LINE_AA);
    //center.x = StartX;	center.y = StartY;
    //circle(frame, center, r_v, Scalar(0, 255, 0), 1, LINE_AA);

    //center.x = obs_pos[0][0];	center.y = obs_pos[0][1];
    //circle(frame, center, obs_sz[0], Scalar(0, 0, 255), 1, LINE_AA);
    //center.x = obs_pos[1][0];	center.y = obs_pos[1][1];
    //circle(frame, center, obs_sz[1], Scalar(0, 0, 255), 1, LINE_AA);
    //center.x = obs_pos[2][0];	center.y = obs_pos[2][1];
    //circle(frame, center, obs_sz[2], Scalar(0, 0, 255), 1, LINE_AA);

    std::vector<cv::Rect> boundingBoxes;
    cout << "Select the calibration board of the vehicle.Press enter to confirm the selection\n" << endl;
    cout << "Press ENTER to confirm the selection\n" << endl;
    cout << "Press ESC to proceed to the next step\n " << endl;
    selectROIs("Tracker", frame, boundingBoxes);
    // quit when the tracked object(s) is not provided
    if (boundingBoxes.size() < 1)
        return 0;

    // initialize the tracker
    for (const auto& boundingBox : boundingBoxes) {
        multiTracker->add(cv::TrackerKCF::create(), frame, boundingBox);
    }

    //載具heading
    cv::cvtColor(frame, subFrame, cv::COLOR_BGR2GRAY);
    cv::Rect frameForCanny = getSafeRect(frame.cols, frame.rows, boundingBoxes[0]);
    subFrame = subFrame(frameForCanny);
    cv::equalizeHist(subFrame, subFrame);
    cv::resize(subFrame, bigDst, cv::Size(0, 0), 2.0, 2.0, cv::INTER_CUBIC);
    cv::resize(subFrame, subFrame, cv::Size(0, 0), 2.0, 2.0, cv::INTER_CUBIC);
    cv::Canny(bigDst, bigDst, 170, 255, 3, true);
    cv::HoughLinesP(bigDst, lines, 1, CV_PI / 180.0, 50, 50, 10);
    cv::cvtColor(bigDst, markedCanny, cv::COLOR_GRAY2BGR);
    cv::Point2d attitude = getAttitude(lines, cv::Point2d(bigDst.cols / 2.0, bigDst.rows / 2.0));
    vhc_hd[0] = attitude.x;	vhc_hd[1] = attitude.y;
    double theta = atan2(vhc_hd[1], vhc_hd[0]);
    if (vhc_hd[1] < 0) {
        theta = 2 * PI + theta;
    }
    else if (vhc_hd[0] == 0 && vhc_hd[1] > 0) {
        theta = PI / 2;
    }
    else if (vhc_hd[0] == 0 && vhc_hd[1] < 0) {
        theta = 3 * PI / 2;
    }

    //取得物件資訊
    boundingBoxes[0] = multiTracker->getObjects()[0];	//vehicle
    vhc[0] = boundingBoxes[0].x + boundingBoxes[0].width / 2.;
    vhc[1] = boundingBoxes[0].y + boundingBoxes[0].height / 2.;
    StartX = (int)(vhc[0] + cos(theta) * 3); StartY = (int)(vhc[1] + sin(theta) * 3);
    printf("vhc: %lf %lf\n", vhc[0], vhc[1]);
    printf("start: %d %d\n", StartX, StartY);

    boundingBoxes[1] = multiTracker->getObjects()[1];	//obs1 (r=73
    obs_pos[0][0] = (int)(boundingBoxes[1].x + boundingBoxes[1].width / 2);
    obs_pos[0][1] = (int)(boundingBoxes[1].y + boundingBoxes[1].height / 2);
    printf("obs 1: %d %d\n", obs_pos[0][0], obs_pos[0][1]);

    boundingBoxes[2] = multiTracker->getObjects()[2];	//obs1 (r=56
    obs_pos[1][0] = (int)(boundingBoxes[2].x + boundingBoxes[2].width / 2);
    obs_pos[1][1] = (int)(boundingBoxes[2].y + boundingBoxes[2].height / 2);
    printf("obs 2: %d %d\n", obs_pos[1][0], obs_pos[1][1]);

    boundingBoxes[3] = multiTracker->getObjects()[3];	//obs1 (r=50
    obs_pos[2][0] = (int)(boundingBoxes[3].x + boundingBoxes[3].width / 2);
    obs_pos[2][1] = (int)(boundingBoxes[3].y + boundingBoxes[3].height / 2);
    printf("obs 3: %d %d\n", obs_pos[2][0], obs_pos[2][1]);

    boundingBoxes[4] = multiTracker->getObjects()[4];	//goal
    goal[0] = (int)(boundingBoxes[4].x + boundingBoxes[4].width / 2);
    goal[1] = (int)(boundingBoxes[4].y + boundingBoxes[4].height / 2);
    printf("goal: %d %d\n", goal[0], goal[1]);

    for (int i = 0; i < image_size.height; i++) {
        for (int j = 0; j < image_size.width; j++) {
            for (int k = 0; k < obs_num; k++) {
                if (sqrt(pow(j - obs_pos[k][0], 2) + pow(i - obs_pos[k][1], 2)) < obs_sz[k] + turning_coeff[0] + 3)
                    MAP[i][j] = 1;
            }
        }
    }


    //boundingBoxes[1] = multiTracker->getObjects()[1];	//endl
    
//double obstacle[2] = { 0.0 };
//obstacle[0] = boundingBoxes[1].x + boundingBoxes[1].width / 2.;
//obstacle[1] = boundingBoxes[1].y + boundingBoxes[1].height / 2.;

    printf("Path planning has started...");
    if(mode == true)
        printf(" (Modified a*)\n");
    else
        printf(" (Original a*)\n");

    int** path = NULL;
    int* path_len = (int*)malloc(sizeof(int));
    if (mode == true) 
    {
        path = modi_astar(StartX, StartY, MAP, goal, Connecting_Distance, obs_pos, obs_sz, obs_num, turning_coeff, height, width, path_len, cap, image);
    }
    else
    {
        path = org_astar(StartX, StartY, MAP, goal, Connecting_Distance, obs_pos, obs_sz, obs_num, turning_coeff, height, width, path_len, cap, image);
    }

    //for(int i = 0; i < *path_len; i++)
    //    printf("%d %d %d\n", path[i][0], path[i][1], i);
    
    //b-spline
    double** tracking_path = NULL;
    tracking_path = b_spline(path, *path_len);

    *path_len = (*path_len + 1) * 4;
    //printf("%d\n", *path_len);
    reverse_arr(tracking_path, *path_len);
    
    printf("Path planning has completed\n");
    for (int i = 0; i < (*path_len); i++)
        printf("%lf %lf %d\n", tracking_path[i][0], tracking_path[i][1], i);

    //fuzzy參數設定
    double drmf_paras[15] = {
    3, 3, 6,   
    3, 6, 9, 
    6, 9, 12,
    9, 12, 15,
    12, 15, 15
    };

    double armf_paras[15] = {
    -PI / 2, -PI / 2, -PI / 4,
    -PI / 2, -PI / 4, 0,
    -PI / 4, 0, PI / 4,
    0, PI / 4, PI / 2,
    PI / 4, PI / 2, PI / 2
    };
    
    double old_target[2] = {0.0};
    double target[2] = { 0.0 };
    double heading_ang[2] = { 0.0 };;
    double dr = 0.;
    double ar = 0.;
    double new_hd[2] = { 0.0 };
    double heading_ar = 0.;
    double xdot = 0.0;
    double ydot = 0.0;
    double thetadot = 0.0;
    double x_c_dot = 0.0;
    double y_c_dot = 0.0;
    double v_control = 0.0;
    double psi_control = 0.0;

    

    int pre_val = 6;
    double** pre_pos = (double**)malloc(pre_val * sizeof(double*));
    for (int i = 0; i < pre_val; i++)
        pre_pos[i] = (double*)malloc(2 * sizeof(double));
    

    for (int i = 0; i < pre_val; i++)             //initialize
    {
        pre_pos[i][0] = cos(theta + PI) * (pre_val - i) + vhc[0];
        pre_pos[i][1] = sin(theta + PI) * (pre_val - i) + vhc[1];
    }

  /*  double theta = atan2(tracking_path[1][1] - tracking_path[0][1], tracking_path[1][0] - tracking_path[0][0]);
    if ((tracking_path[1][1] - tracking_path[0][1]) < 0) {
        theta = 2 * PI + theta;
    }
    else if ((tracking_path[1][0] - tracking_path[0][0]) == 0 && (tracking_path[1][1] - tracking_path[0][1]) > 0) {
        theta = PI / 2;
    }
    else if ((tracking_path[1][0] - tracking_path[0][0]) == 0 && (tracking_path[1][1] - tracking_path[0][1]) < 0) {
        theta = 3 * PI / 2;
    }*/

    int myepoch = 1;
    int counter = 1;
    target[0] = tracking_path[pre_val][0];
    target[1] = tracking_path[pre_val][1];

    Mat tracking_frame(480, 640, CV_64FC4, cv::Scalar(255, 255, 255));

    //fuzzy-control
    printf("Start tracking\n");
    while (cap.read(image) && cap.isOpened()) {
        // 終止條件
        if (myepoch > *path_len) { 
            break;
        }

        // Check image
        if (image.empty()) 
            break;
        cv::remap(image, frame, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
        // Update the multi tracker and draw rectangles around objects
        multiTracker->update(frame);

        //// draw the tracked object
        //for (unsigned i = 0; i < multiTracker->getObjects().size(); i++) {
        //    rectangle(frame, multiTracker->getObjects()[i], Scalar(255, 0, 0), 2, 1);
        //}

        //取得物件資訊
        boundingBoxes[0] = multiTracker->getObjects()[0];	//vehicle
        vhc[0] = boundingBoxes[0].x + boundingBoxes[0].width / 2;
        vhc[1] = boundingBoxes[0].y + boundingBoxes[0].height / 2;
        printf("%lf %lf", vhc[0], vhc[1]);
        if (myepoch > 20)
        {
            /*if (((vhc[0] - preStartPose[0]) * (target[0] - preStartPose[0]) + (vhc[1] - preStartPose[1]) * (target[1] - preStartPose[1])) < 0)
                continue;*/
            if (vhc[0] == pre_pos[counter % pre_val][0] && vhc[1] == pre_pos[counter % pre_val][1])
            {
                myRacer.setVelDS(-10, v_control);
                continue; 
            }
                               
        }
        
    
printf("vhc_pos: %lf %lf\n", vhc[0], vhc[1]);
        //終止條件
        //發生碰撞
        if (collision_check(obs_pos, obs_sz, vhc, r_v, obs_num))
        {
            printf("COLLISION!!!!!\n");
            break;
        }

        //到達終點
        if (sqrt(pow((vhc[1] - goal[1]), 2) + pow((vhc[0] - goal[0]), 2)) < 20) 
        {
            printf("Arrive\n");
            break;
        }


        printf("target: %lf %lf\n", target[0], target[1]);
        //printf("preStartPose: %lf %lf\n", preStartPose[0], preStartPose[1]);
        printf("tracking_path: %lf %lf\n", tracking_path[myepoch][0], tracking_path[myepoch][1]);
        printf("vhc: %lf %lf\n", vhc[0], vhc[1]);
        if (myepoch > 20)
        {
            
        /*    while ((abs(tracking_path[myepoch][1] - vhc[1]) < 2) || (abs(tracking_path[myepoch][0] - vhc[0]) < 2))
                myepoch++;*/
            while((sqrt(pow((tracking_path[myepoch][0] - vhc[0]), 2) + pow((tracking_path[myepoch][1] - vhc[1]), 2)) < 2))
                myepoch++;
            /*while (((target[0] - preStartPose[0]) * (tracking_path[myepoch][0] - vhc[0]) + (target[1] - preStartPose[1]) * (tracking_path[myepoch][1] - vhc[1])) <= 0)
                myepoch++;*/
  
        }

       /* for (int check = 1; check <= 100; check++) 
        {
            if (tracking_path[myepoch - check][0] != tracking_path[myepoch][0] || tracking_path[myepoch - check][1] != tracking_path[myepoch][1]) {
                old_target[0] = tracking_path[myepoch - check][0];
                old_target[1] = tracking_path[myepoch - check][1];
                break;
            }
        }*/
        
        target[0] = tracking_path[myepoch][0];
        target[1] = tracking_path[myepoch][1];
printf("target: %lf %lf\n", target[0], target[1]);
        heading_ang[0] = target[0] - old_target[0];
        heading_ang[1] = target[1] - old_target[1];
        dr = sqrt(pow(target[0] - vhc[0], 2) + pow(target[1] - vhc[1], 2));
//printf("preStartPose: %lf %lf;    vhc: %lf %lf;    target: %lf %lf      ", preStartPose[0], preStartPose[1], vhc[0], vhc[1], target[0], target[1]);
        ar = angletest(pre_pos[counter % pre_val], vhc, target);
//printf("ar: %lf;\n", ar);
     /*   new_hd[0] = vhc[0] + target[0] - old_target[0];
        new_hd[1] = vhc[1] + target[1] - old_target[1];*/
//printf("preStartPose: %lf %lf;    vhc: %lf %lf;    new_hd: %lf %lf      ", preStartPose[0], preStartPose[1], vhc[0], vhc[1], new_hd[0], new_hd[1]);
        //heading_ar = angletest(preStartPose, vhc, new_hd);
//printf("heading_ar: %lf\n", heading_ar);
        //ar = ar + (1 / (0.2 * dr)) * heading_ar;
printf("dr: %lf; ar: %lf\n", dr, ar);
        
        
        pre_pos[counter % pre_val][0] = vhc[0];
        pre_pos[counter % pre_val][1] = vhc[1];
        counter++;

        

        int count = 0, m = 0, n = 0;
        double inputDis_list[2][2] = { {0.0, 0.0}, {0.0, 0.0} };
        double inputAng_list[2][2] = { {0.0, 0.0}, {0.0, 0.0} };

        while (count < 15) {
            double disweigh, angweigh;
            int dismf, angmf;


            if (count == 0) {
                disTrapL(dr, drmf_paras[count + 1], drmf_paras[count + 2], &disweigh, &dismf);
            }
            else if (count == 12) {
                disTrapR(dr, drmf_paras[count], drmf_paras[count + 1], &disweigh, &dismf);
            }
            else {
                disTri(dr, drmf_paras[count], drmf_paras[count + 1], drmf_paras[count + 2], &disweigh, &dismf);
            }

            if (count == 0) {
                angTrapL(ar, armf_paras[count + 1], armf_paras[count + 2], &angweigh, &angmf);
            }
            else if (count == 12) {
                angTrapR(ar, armf_paras[count], armf_paras[count + 1], &angweigh, &angmf);
            }
            else {
                angTri(ar, armf_paras[count], armf_paras[count + 1], armf_paras[count + 2], &angweigh, &angmf);
            }
//printf("%lf %lf %d %d\n", disweigh, angweigh, dismf, angmf);
            if (disweigh != 0) {
                inputDis_list[m][0] = disweigh;
                inputDis_list[m][1] = dismf;
                m++;
            }
            if (angweigh != 0) {
                inputAng_list[n][0] = angweigh;
                inputAng_list[n][1] = angmf;
                n++;
            }
            count += 3;
        }

        double accR = 0.0, accL = 0.0, accEpoch = 0.0;
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                double yweigh = fmin(inputDis_list[i][0], inputAng_list[j][0]);
                int Rmf, Lmf, time;
                rule(inputDis_list[i][1], inputAng_list[j][1], &Rmf, &Lmf, &time);
//printf("Rmf: %d; Lmf: %d; time: %d\n", Rmf, Lmf, time);
                defuzzy(&accR, &accL, &accEpoch, yweigh, Rmf, Lmf, time);
            }
        }

        if(sqrt(pow((vhc[1] - tracking_path[myepoch + (int)round(accEpoch)][1]), 2) + pow((vhc[0] - tracking_path[myepoch + (int)round(accEpoch)][0]), 2)) < 5)
        //if (sqrt(pow((target[1] - tracking_path[myepoch + (int)round(accEpoch)][1]), 2) + pow((target[0] - tracking_path[myepoch + (int)round(accEpoch)][0]), 2)) < 5)
            myepoch += round(accEpoch);
printf("accR: %lf; accL: %lf; accEpoch: %lf\n", accR, accL, accEpoch);            

        //載具heading
        cv::cvtColor(frame, subFrame, cv::COLOR_BGR2GRAY);
        cv::Rect frameForCanny = getSafeRect(frame.cols, frame.rows, boundingBoxes[0]);
        subFrame = subFrame(frameForCanny);
        cv::equalizeHist(subFrame, subFrame);
        cv::resize(subFrame, bigDst, cv::Size(0, 0), 2.0, 2.0, cv::INTER_CUBIC);
        cv::resize(subFrame, subFrame, cv::Size(0, 0), 2.0, 2.0, cv::INTER_CUBIC);
        cv::Canny(bigDst, bigDst, 170, 255, 3, true);
        cv::HoughLinesP(bigDst, lines, 1, CV_PI / 180.0, 50, 50, 10);
        cv::cvtColor(bigDst, markedCanny, cv::COLOR_GRAY2BGR);
        //cv::rectangle(frame, objects[i], colors[i], 2, 8);

        //cv::rectangle(frame, object, colors[i], 2, 8);
        boundingBoxes[0] = multiTracker->getObjects()[0];
        //cv::circle(frame, cv::Point_<int>(objects[i].x + objects[i].width / 2, objects[i].y + objects[i].height / 2), 2, colors[i], CV_FILLED, 8);
//std::cout << "\nPosition of car in X-axis :" << boundingBoxes[0].x + boundingBoxes[0].width / 2 << std::endl;
//std::cout << "\nPosition of car in Y-axis :" << boundingBoxes[0].y + boundingBoxes[0].height / 2 << std::endl;
        center.x = boundingBoxes[0].x + boundingBoxes[0].width / 2.;
        center.y = boundingBoxes[0].y + boundingBoxes[0].height / 2.;
        //cv::circle(tracking_frame, center, 2, colors[i], CV_FILLED, 8);

        cv::Point2d attitude = getAttitude(lines, cv::Point2d(bigDst.cols / 2.0, bigDst.rows / 2.0));
        //std::cout << attitude << std::endl;
        //heading = center - (cv::Point2d)(attitude * 30.0);

        vhc_hd[0] = attitude.x;	vhc_hd[1] = attitude.y;
        
        heading = center + (cv::Point2d)(attitude * 30.0);
        //std::cout << heading << std::endl;
        cv::arrowedLine(frame, center, heading, Scalar(0, 0, 255), 1, CV_AA);

        theta = atan2(vhc_hd[1], vhc_hd[0]);
        if (vhc_hd[1] < 0) {
            theta = 2 * PI + theta;
        }
        else if (vhc_hd[0] == 0 && vhc_hd[1] > 0) {
            theta = PI / 2;
        }
        else if (vhc_hd[0] == 0 && vhc_hd[1] < 0) {
            theta = 3 * PI / 2;
        }
printf("heading angle: %lf\n", theta);

        xdot = rw / 2 * cos(theta) * (accL + accR);
        ydot = rw / 2 * sin(theta) * (accL + accR);
        thetadot = rw / L_c * (accL - accR);
        v = sqrt(pow(xdot, 2) + pow(ydot, 2));
        psi = atan2((L_c * thetadot), v);
        if (fabs(psi) > 25 * PI / 180) {
            if (psi > 0) {
                psi = 25 * PI / 180;
            }
            else {
                psi = -25 * PI / 180;
            }
        }
printf("psi: %lf;   v: %lf\n", psi, v);
        v_control = 50;
        psi_control = steering_val(psi*180/PI);
        myRacer.setVelDS(psi_control, v_control);
printf("%lf %lf\n", psi_control, v_control);
printf("\n");
        //Sleep(500);


        //畫圖
        center.x = goal[0];	center.y = goal[1];				//Map_goal	
        circle(frame, center, 2, Scalar(0, 255, 0), 1, LINE_AA);
        center.x = vhc[0];	center.y = vhc[1];				//vehicle	
        circle(frame, center, r_v, Scalar(0, 255, 0), 1, LINE_AA);

        center.x = obs_pos[0][0];	center.y = obs_pos[0][1];				//obs1		
        circle(frame, center, obs_sz[0], Scalar(0, 255, 0), 1, LINE_AA);
        circle(frame, center, 2, Scalar(0, 255, 0), 1, LINE_AA);
        center.x = obs_pos[1][0];	center.y = obs_pos[1][1];				//obs2	
        circle(frame, center, obs_sz[1], Scalar(0, 255, 0), 1, LINE_AA);
        circle(frame, center, 2, Scalar(0, 255, 0), 1, LINE_AA);
        center.x = obs_pos[2][0];	center.y = obs_pos[2][1];				//obs2	
        circle(frame, center, obs_sz[2], Scalar(0, 255, 0), 1, LINE_AA);
        circle(frame, center, 2, Scalar(0, 255, 0), 1, LINE_AA);
        //center.x = target[0];	center.y = target[1];				//target
        //circle(frame, center, 2, Scalar(255, 0, 0), 1, LINE_AA);
        
        //軌跡
        center.x = goal[0];	center.y = goal[1];				//Map_goal	
        circle(tracking_frame, center, 2, Scalar(0, 0, 255), 1, LINE_AA);
        center.x = vhc[0];	center.y = vhc[1];				//F1p		
        circle(tracking_frame, center, 1, Scalar(0, 0, 255), 1, LINE_AA);

        namedWindow("Path");
        namedWindow("Tracker");
        imshow("Path", tracking_frame);
        //imwrite("Path.jpg", tracking_frame);
        imshow("Tracker", frame);
        if (cv::waitKey(1) == 27) break;

        //break;

    }


	return 0;
	



}


//------------------------------------------Lifa----------------------------------------------------
//int main(int argc, char** argv) {
//	//car settinng
////int Angle[3], AngularVelocity[3], Acceleration[3], Magnetic[3];
//    clock_t new_t, pre_t;
//	char CAR_IP[64] = DEF_CAR_IP;
//
//	//chose car
//	if (argc > 1) {
//		strcpy_s(CAR_IP, argv[1]);
//		if (CAR_IP[0] == 0) {
//			strcpy_s(CAR_IP, DEF_CAR_IP);
//		}
//	}
//
//	if (myRacer.init() < 0) {
//		printf("INIT ERR\r\n");
//	}
//	else
//	{
//		myRacer.debug = true;
//		printf("CAR IP = %s\r\n", CAR_IP);
//		myRacer.select(CAR_IP);
//		if (!waitCarReady()) {
//			return 0;
//		}
//	}
//
//
//    double time_shift[3] = {0.0202, 0.1957, 0.0808};
//    double* X = (double*)malloc(300 * sizeof(double));
//    double* Y = (double*)malloc(300 * sizeof(double));
//    int del = 100;
//    for (int j = 0; j < 3; j++)
//    {
//        for (int i = 0; i < del; i++)
//        {
//            scanf_s("%lf ", &(X[j * del + i]));
//        }
//    }
//printf("X ok\n");
//    for (int j = 0; j < 3; j++)
//    {
//        for (int i = 0; i < del; i++)
//        {
//            scanf_s("%lf ", &(Y[j * del + i]));
//        }
//    }
//printf("Y ok\n"); 
//
////myRacer.setVelDS(0.31, -100);
//    for (int j = 0; j < 3; j++)
//    {
//        for (int i = 0; i < del; i++)
//        {
//            pre_t = clock();
//            new_t = clock();
//            while ((double)(new_t - pre_t) / CLOCKS_PER_SEC < time_shift[j])
//            {
//                myRacer.setVelDS(Y[j * del + i], X[j * del + i]);
//                new_t = clock();
//printf("%lf %lf \n", Y[j * del + i], X[j * del + i]);
//            }
//            
//printf("%d %d\n", j, i);
//        }
//    }
//
//
//
//
//}
