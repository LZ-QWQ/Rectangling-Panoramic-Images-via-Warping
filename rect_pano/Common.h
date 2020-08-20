#pragma once

#define INF 1e8
#define PI 3.14159265358979323846

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include<iostream>
#include<vector>
#include<algorithm>
#include"lsd.h"
#include <Eigen/Sparse>
#include<Eigen/Dense>
#include<cmath>
#include "GL/glut.h"

typedef cv::Mat CVMat;
typedef cv::Vec3b colorPixel;

typedef Eigen::SparseMatrix<double> SparseMatrixD;//列优先
typedef Eigen::SparseMatrix<double, Eigen::RowMajor> SpareseMatrixD_Row;//行优先
typedef Eigen::VectorXd VectorXd;
typedef Eigen::MatrixXd MatrixXd;
typedef Eigen::Vector2d Vector2d;
typedef Eigen::Vector2i Vector2i;
typedef Eigen::MatrixXi MatrixXi;
typedef Eigen::Matrix2d Matrix2d;
//typedef Eigen::Triplet<double> T;
typedef Eigen::SimplicialCholesky<SparseMatrixD> CSolve;

using namespace std;

//图形大小即mesh等参数
struct Config {//运行前配置
	int rows;
	int cols;
	int meshNumRow;
	int meshNumCol;
	int meshQuadRow;
	int meshQuadCol;
	double rowPermesh;
	double colPermesh;
	Config(int rows, int cols, int meshNumRow, int meshNumCol) {
		this->rows = rows;
		this->cols = cols;
		this->meshNumRow = meshNumRow;
		this->meshNumCol = meshNumCol;
		this->meshQuadCol = meshNumCol - 1;
		this->meshQuadRow = meshNumRow - 1;
		this->rowPermesh = double(rows - 1) / (meshNumRow - 1);//真的是精准计算啊...
		this->colPermesh = double(cols - 1) / (meshNumCol - 1);
	}
};

//整型坐标
struct Coordinate {
	int row;
	int col;

	bool operator==(const Coordinate& rhs) const {
		return (row == rhs.row && col == rhs.col);
	}
	bool operator<(const Coordinate& rhs) const {
		// this operator is used to determine equality, so it must use both x and y
		if (row < rhs.row) {
			return true;
		}
		if (row > rhs.row) {
			return false;
		}
		return col < rhs.col;
	}
	Coordinate() { row = 0; col = 0; };
	Coordinate(int setRow, int setCol) { row = setRow; col = setCol; };
};

//浮点坐标
struct CoordinateDouble {
	double row;
	double col;

	bool operator==(const CoordinateDouble& rhs) const {
		return (row == rhs.row && col == rhs.col);
	}
	bool operator<(const CoordinateDouble& rhs) const {
		// this operator is used to determine equality, so it must use both x and y
		if (row < rhs.row) {
			return true;
		}
		if (row > rhs.row) {
			return false;
		}
		return col < rhs.col;
	}

	CoordinateDouble operator+(const CoordinateDouble& b)
	{
		CoordinateDouble temp;
		temp.row = row + b.row;
		temp.col = col + b.col;
		return temp;
	}
	CoordinateDouble operator-(const CoordinateDouble& b)
	{
		CoordinateDouble temp;
		temp.row = row - b.row;
		temp.col = col - b.col;
		return temp;
	}

	friend ostream &operator<<(ostream &stream, const CoordinateDouble &p){
		stream << "("<<p.col<<","<<p.row<<")";
		return stream;
	}
	CoordinateDouble() { row = 0; col = 0; };
	CoordinateDouble(double setRow, double setCol) { row = setRow; col = setCol; };
};

//线~
struct LineD {
	double row1, col1;
	double row2, col2;
	LineD(double row1, double col1, double row2, double col2) {
		this->row1 = row1;
		this->row2 = row2;
		this->col1 = col1;
		this->col2 = col2;
	}
	LineD() { row1 = 0; col1 = 0; row2 = 0; col2 = 0; }
	LineD(CoordinateDouble p1, CoordinateDouble p2) { row1 = p1.row; row2 = p2.row; col1 = p1.col; col2 = p2.col; }

};

CVMat Mask_contour(const CVMat src);
vector<vector<CoordinateDouble>> vector_to_mesh(VectorXd x, Config config);
SpareseMatrixD_Row row_stack(SparseMatrixD origin, SpareseMatrixD_Row diag);
SpareseMatrixD_Row row_stack(SpareseMatrixD_Row origin, SpareseMatrixD_Row diag);
MatrixXd row_stack(MatrixXd mat1, MatrixXd mat2);
MatrixXd col_stack(MatrixXd mat1, MatrixXd mat2);
void DrawLine(CVMat& img, CoordinateDouble coordstart, CoordinateDouble coordend);
void DrawLine(CVMat& img, LineD line);
void draw_savemesh(const CVMat src, string filename, vector<vector<CoordinateDouble>> mesh, Config config);
void enlarge_mesh(vector<vector<CoordinateDouble>>& mesh, double enlarge_x, double enlarge_y, Config config);

void compute_scaling(double &sx_avg, double& sy_avg, const vector<vector<CoordinateDouble>> mesh,
	const vector<vector<CoordinateDouble>> outputmesh, const Config config);

GLuint matToTexture(cv::Mat mat, GLenum minFilter = GL_LINEAR, GLenum magFilter = GL_LINEAR, GLenum wrapFilter = GL_CLAMP);