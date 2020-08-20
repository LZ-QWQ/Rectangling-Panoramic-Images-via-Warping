#include"Common.h"



//泛洪填充
void fillHole(const CVMat srcBw, CVMat &dstBw) {
	cv::Size m_Size = srcBw.size();
	CVMat Temp = CVMat::zeros(m_Size.height + 2, m_Size.width + 2, srcBw.type());//延展图像  
	srcBw.copyTo(Temp(cv::Range::Range(1, m_Size.height + 1), cv::Range::Range(1, m_Size.width + 1)));

	cv::floodFill(Temp, cv::Point(0, 0), cv::Scalar(255));//默认4

	CVMat cutImg;//裁剪延展的图像  
	Temp(cv::Range::Range(1, m_Size.height + 1), cv::Range::Range(1, m_Size.width + 1)).copyTo(cutImg);

	dstBw = srcBw | (~cutImg);
}

//构造mask
CVMat Mask_contour(const CVMat src) 
{
	CVMat bw;
	CVMat src_copy;
	src.copyTo(src_copy);
	cv::cvtColor(src, src_copy, cv::COLOR_BGR2GRAY);

	uchar thr = 252;
	CVMat mask = CVMat::zeros(src_copy.size(), CV_8UC1);
	for (int row = 0; row < src_copy.rows; row++) {
		for (int col = 0; col < src_copy.cols; col++) {
			if (src_copy.at<uchar>(row, col)<thr) {
				mask.at<uchar>(row, col) = 255;
			}
		}
	}
	//泛洪、按位取反、或，就可以保证bw中255为有像素的部分😓(再取反一次就是0才是~)
	fillHole(mask, bw);

	bw = ~bw;//按位取反，，绝了
	
	//这里的膨胀腐蚀操作是为了啥呢，我感觉效果一般般啊，先注释掉吧，，
	/*
	//https://www.cnblogs.com/ssyfj/p/9276999.html 我在这里了解了下膨胀腐蚀....
	//https://blog.csdn.net/hanshanbuleng/article/details/80657148 开运算与闭运算，这里做的就是闭运算，填平小孔？？绝了
	CVMat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
	//cout << element << endl;
	CVMat dilate_out;//膨胀

	//cout << bw << endl;
	cv::dilate(bw, dilate_out, element);
	cv::dilate(dilate_out, dilate_out, element);
	cv::dilate(dilate_out, dilate_out, element);
	//cout << dilate_out << endl;

	CVMat erode_out;//腐蚀
	erode(dilate_out, erode_out, element);
	//cout << erode_out <<endl;
	return erode_out;*/
	return bw;
}

//把解转为mesh
vector<vector<CoordinateDouble>> vector_to_mesh(VectorXd x, Config config) {
	int numMeshRow = config.meshNumRow;
	int numMeshCol = config.meshNumCol;
	vector<vector<CoordinateDouble>> mesh;
	//row为y轴，col为x轴 Vq=[y0,x0,y1,x1...]
	for (int row = 0; row < numMeshRow; row++) 
	{
		vector<CoordinateDouble> meshRow;
		for (int col = 0; col < numMeshCol; col++)
		{
			int xid = (row * numMeshCol + col) * 2;
			CoordinateDouble coord;
			coord.row = x(xid + 1);
			coord.col = x(xid);
			meshRow.push_back(coord);
		}
		mesh.push_back(meshRow);
	}
	return mesh;
}

//按行堆叠两个矩阵
SpareseMatrixD_Row row_stack(SparseMatrixD origin, SpareseMatrixD_Row diag) {
	SpareseMatrixD_Row res(origin.rows() + diag.rows(), origin.cols());
	res.topRows(origin.rows()) = origin;
	res.bottomRows(diag.rows()) = diag;
	return res;
}
SpareseMatrixD_Row row_stack(SpareseMatrixD_Row origin, SpareseMatrixD_Row diag) {
	SpareseMatrixD_Row res(origin.rows() + diag.rows(), origin.cols());
	res.topRows(origin.rows()) = origin;
	res.bottomRows(diag.rows()) = diag;
	return res;
}
MatrixXd row_stack(MatrixXd mat1, MatrixXd mat2) {
	MatrixXd res(mat1.rows() + mat2.rows(), mat1.cols());
	res.topRows(mat1.rows()) = mat1;
	res.bottomRows(mat2.rows()) = mat2;
	return res;
}
//按列堆叠
MatrixXd col_stack(MatrixXd mat1, MatrixXd mat2) {
	MatrixXd res(mat1.rows(), mat1.cols() + mat2.cols());
	res.leftCols(mat1.cols()) = mat1;
	res.rightCols(mat2.cols()) = mat2;
	return res;
}

//画线1 直接int
void DrawLine(CVMat& img, CoordinateDouble coordstart, CoordinateDouble coordend) 
{
	cv::Point start((int)coordstart.col, (int)coordstart.row);
	cv::Point end((int)coordend.col, (int)coordend.row);
	int thickness = 1;
	int lineType = cv::LINE_AA;//貌似是反锯齿
	cv::line(img, start, end, cv::Scalar(0, 255, 0), thickness, lineType);
}
//画线2 直接int
void DrawLine(CVMat& img, LineD line) {
	cv::Point start((int)line.col1, (int)line.row1);
	cv::Point end((int)line.col2, (int)line.row2);
	int thickness = 1;
	int lineType = cv::LINE_AA;//貌似是反锯齿
	cv::line(img, start, end, cv::Scalar(0, 255, 0), thickness, lineType);
}
//画mesh
void draw_savemesh(const CVMat src, string filename,vector<vector<CoordinateDouble>> mesh, Config config) 
{
	CVMat src_copy;
	src.copyTo(src_copy);
	int meshNumRow = config.meshNumRow;
	int meshNumCol = config.meshNumCol;

	for (int row = 0; row < meshNumRow; row++) {
		for (int col = 0; col < meshNumCol; col++) {
			CoordinateDouble now = mesh[row][col];
			if (row == meshNumRow - 1 && col<meshNumCol - 1) {
				CoordinateDouble right = mesh[row][col + 1];
				DrawLine(src_copy, now, right);
			}
			else if (row < meshNumRow - 1 && col == meshNumCol - 1) {
				CoordinateDouble down = mesh[row + 1][col];
				DrawLine(src_copy, now, down);
			}
			else if (row == meshNumRow - 1 && col == meshNumCol - 1);
			else 
			{//row < meshNumRow - 1 && col < meshNumCol - 1
				CoordinateDouble right = mesh[row][col + 1];
				DrawLine(src_copy, now, right);
				CoordinateDouble down = mesh[row + 1][col];
				DrawLine(src_copy, now, down);
			}
		}
	}
	/*cv::namedWindow("Mesh", cv::WINDOW_AUTOSIZE);
	cv::imshow("Mesh", src_copy);
	cv::waitKey(0);*/
	cv::imwrite(filename, src_copy);
}

//根据x y的因子调整mesh
void enlarge_mesh(vector<vector<CoordinateDouble>>& mesh, double enlarge_x,double enlarge_y, Config config) 
{
	int numMeshRow = config.meshNumRow;
	int numMeshCol = config.meshNumCol;
	for (int row = 0; row < numMeshRow; row++) 
	{
		for (int col = 0; col < numMeshCol; col++) 
		{
			CoordinateDouble &coord = mesh[row][col];
			coord.row = coord.row * enlarge_y;
			coord.col = coord.col * enlarge_x;
		}
	}
};

//计算论文中最后提到的缩放因子
void compute_scaling(double &sx_avg, double& sy_avg, const vector<vector<CoordinateDouble>> mesh, 
	const vector<vector<CoordinateDouble>> outputmesh,const Config config)
{
	//row为y轴，col为x轴
	int numQuadRow = config.meshQuadRow;
	int numQuadCol = config.meshQuadRow;
	int sx = 0, sy = 0;
	for (int row = 0; row < numQuadRow; row++)
	{
		for (int col = 0; col < numQuadCol; col++)
		{
			CoordinateDouble p0 = mesh[row][col];//左上
			CoordinateDouble p1 = mesh[row][col + 1];//右上
			CoordinateDouble p2 = mesh[row + 1][col];//左下
			CoordinateDouble p3 = mesh[row + 1][col + 1];//右下
			
			CoordinateDouble p0_out = outputmesh[row][col];//左上
			CoordinateDouble p1_out = outputmesh[row][col + 1];//右上
			CoordinateDouble p2_out = outputmesh[row + 1][col];//左下
			CoordinateDouble p3_out = outputmesh[row + 1][col + 1];//右下

			CVMat A = (cv::Mat_<double>(1, 4) << p0.row, p1.row, p2.row, p3.row);
			CVMat B = (cv::Mat_<double>(1, 4) << p0_out.row, p1_out.row, p2_out.row, p3_out.row);
			double max_temp, min_temp;
			double max_temp_out, min_temp_out;
			cv::minMaxIdx(A, &max_temp, &min_temp);
			cv::minMaxIdx(B, &max_temp_out, &min_temp_out);
			sy += (max_temp_out - min_temp_out) / (max_temp - min_temp);
			
			CVMat C = (cv::Mat_<double>(1, 4) << p0.col, p1.col, p2.col, p3.col);
			CVMat D = (cv::Mat_<double>(1, 4) << p0_out.col, p1_out.col, p2_out.col, p3_out.col);
			max_temp = 0; min_temp = 0; max_temp_out = 0; min_temp_out = 0;
			cv::minMaxIdx(C, &max_temp, &min_temp);
			cv::minMaxIdx(D, &max_temp_out, &min_temp_out);
			sx += (max_temp_out - min_temp_out) / (max_temp - min_temp);
		}
	}
	sx_avg = double(sx) / (numQuadRow * numQuadCol);
	sy_avg = double(sy) / (numQuadRow * numQuadCol);
}


