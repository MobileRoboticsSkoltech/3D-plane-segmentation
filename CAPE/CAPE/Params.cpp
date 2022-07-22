//
// Created by michael on 17.07.2022.
//

#include "Params.h"

double DEPTH_SIGMA_COEFF = 0.000001425;
double DEPTH_SIGMA_MARGIN = 10;
double cylinder_score_min = 100;
double cylinder_RANSAC_sqr_max_dist = 0.0225; // square of 15 %

double COS_ANGLE_MAX = cos(M_PI / 12);
double MAX_MERGE_DIST = 50.0;
int PATCH_SIZE = 16;

double MIN_NR_OF_VALID_POINTS_FACTOR = 2; // here it means, min_nr_pts = nr_pts_per_cell/2;
double PLANESEG_MAX_DIFF = 100;
int PLANESEG_JUMP_NUMBER_THRESHOLD_PARAM = 1;

int HISTOGRAM_BINS_PER_COORD_PARAM = 20;
int REGION_GROWING_CANDIDATE_SIZE_THRESHOLD_PARAM = 5;
int REGION_GROWING_CELLS_ACTIVATED_THRESHOLD_PARAM = 4;
double REGION_PLANAR_FITTING_PLANARITY_SCORE_THRESHOLD_PARAM = 100;
int CYLINDER_DETECTION_CELLS_ACTIVATED_THRESHOLD_PARAM = 5;
int REFINEMENT_MULTIPLIER_PARAM = 9;


void readIni(stringstream &params_buff) {
    ifstream fin(params_buff.str());
    if (!fin.is_open()) {
        cout << "[iniLoad] " << params_buff.str() << " not found, using defaults." << endl;
        return;
    }
    while (fin) {
        std::string line;
        std::getline(fin, line);
        if (line.empty() || line[0] == '#') continue;
        std::string key, value;
        size_t eqPos = line.find_first_of('=');
        if (eqPos == std::string::npos || eqPos == 0) {
            std::cout << "[iniLoad] ignore line:" << line << std::endl;
            continue;
        }
        key = line.substr(0, eqPos);
        value = line.substr(eqPos + 1);
        if (key == "depthSigmaCoeff")
            DEPTH_SIGMA_COEFF = strtod(value.c_str(), nullptr);
        else if (key == "depthSigmaMargin")
            DEPTH_SIGMA_MARGIN = strtod(value.c_str(), nullptr);
        else if (key == "cylinderScoreMin")
            cylinder_score_min = strtod(value.c_str(), nullptr);
        else if (key == "cylinderRansacSqrMaxDist")
            cylinder_RANSAC_sqr_max_dist = strtod(value.c_str(), nullptr);
        else if (key == "cosAngleMax")
            COS_ANGLE_MAX = strtod(value.c_str(), nullptr);
        else if (key == "maxMergeDist")
            MAX_MERGE_DIST = strtod(value.c_str(), nullptr);
        else if (key == "patchSize")
            PATCH_SIZE = strtol(value.c_str(), nullptr, 0);
        else if (key == "minNrOfValidPointsOnePerXThreshold")
            MIN_NR_OF_VALID_POINTS_FACTOR = strtod(value.c_str(), nullptr);
        else if (key == "planesegMaxDiff")
            PLANESEG_MAX_DIFF = strtod(value.c_str(), nullptr);
        else if (key == "planarFittingJumpsCounterThresholdParam")
            PLANESEG_JUMP_NUMBER_THRESHOLD_PARAM = strtol(value.c_str(), nullptr, 0);
        else if (key == "histogramBinsPerCoordParam")
            HISTOGRAM_BINS_PER_COORD_PARAM = strtol(value.c_str(), nullptr, 0);
        else if (key == "regionGrowingCandidateSizeThresholdParam")
            REGION_GROWING_CANDIDATE_SIZE_THRESHOLD_PARAM = strtol(value.c_str(), nullptr, 0);
        else if (key == "regionGrowingCellsActivatedThresholdParam")
            REGION_GROWING_CELLS_ACTIVATED_THRESHOLD_PARAM = strtol(value.c_str(), nullptr, 0);
        else if (key == "regionPlanarFittingPlanarityScoreThresholdParam")
            REGION_PLANAR_FITTING_PLANARITY_SCORE_THRESHOLD_PARAM = strtod(value.c_str(), nullptr);
        else if (key == "cylinderDetectionCellsActivatedThreshold")
            CYLINDER_DETECTION_CELLS_ACTIVATED_THRESHOLD_PARAM = strtol(value.c_str(), nullptr, 0);
        else if (key == "refinementMultiplierParam")
            REFINEMENT_MULTIPLIER_PARAM = strtol(value.c_str(), nullptr, 0);
    }
    fin.close();
}
