/*
 * Copyright 2018 Pedro Proenza <p.proenca@surrey.ac.uk> (University of Surrey)
 *
 */
// Kinect 1 sensor model uncertainty according to Khoshelham and Elberink:
#pragma once
#include <cmath>
#include <sstream>
#include <fstream>
#include <iostream>
#define _USE_MATH_DEFINES

using namespace std;

extern double DEPTH_SIGMA_COEFF;
extern double DEPTH_SIGMA_MARGIN;
extern double cylinder_score_min;
extern double cylinder_RANSAC_sqr_max_dist; /* square of 15 %*/

extern double COS_ANGLE_MAX;
extern double MAX_MERGE_DIST;
extern int PATCH_SIZE;

extern double MIN_NR_OF_VALID_POINTS_FACTOR; // it means, min_nr_pts = nr_pts_per_cell/2;
extern double PLANESEG_MAX_DIFF;
extern int PLANESEG_JUMP_NUMBER_THRESHOLD_PARAM;

extern int HISTOGRAM_BINS_PER_COORD_PARAM;
extern int REGION_GROWING_CANDIDATE_SIZE_THRESHOLD_PARAM;
extern int REGION_GROWING_CELLS_ACTIVATED_THRESHOLD_PARAM ;
extern double REGION_PLANAR_FITTING_PLANARITY_SCORE_THRESHOLD_PARAM;
extern int CYLINDER_DETECTION_CELLS_ACTIVATED_THRESHOLD_PARAM;
extern int REFINEMENT_MULTIPLIER_PARAM;


void readIni(stringstream &);
