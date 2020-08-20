#pragma once

#include"Common.h"

enum Border {
	BORDER_TOP = 0,
	BORDER_BOTTOM = 1,
	BORDER_LEFT = 2,
	BORDER_RIGHT = 3
};

enum SeamDirection {
	SEAM_VERTICAL = 0,
	SEAM_HORIZONTAL = 1
};

vector<vector<Coordinate>> Local_wrap(const CVMat src, CVMat& wrap_img, CVMat mask);
CVMat Insert_local_seam(CVMat src, CVMat& seam_img, CVMat& mask, int* seam, SeamDirection seamdirection, pair<int, int> begin_end, bool shiftToend);
int* Get_local_seam(CVMat src, CVMat mask, SeamDirection seamdirection, pair<int, int> begin_end);
int* Get_local_seam_improved(CVMat src, CVMat mask, SeamDirection seamdirection, pair<int, int> begin_end);
vector<vector<Coordinate>> Get_Local_warp_displacement(CVMat& warp_img, CVMat mask);
pair<int, int> Choose_longest_border(CVMat src, CVMat mask, Border& direction);
void warp_mesh_back(vector<vector<CoordinateDouble>>& mesh, vector<vector<Coordinate>> displacementMap, Config config);
vector<vector<CoordinateDouble>> get_rectangle_mesh( Config config);

