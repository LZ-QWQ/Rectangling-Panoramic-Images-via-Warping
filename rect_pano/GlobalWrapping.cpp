#include"GlobalWrapping.h"

//注：下面这两个函数我不懂是咋计算来的 先作废吧。。 从矩阵构造看好像跟逆双线性插值是一样的，只不过就很奇怪咋算的。。
/*
MatrixXd BilinearWeightsToMatrix(BilinearWeights w) {
	MatrixXd mat(2,8);
	double v1w= 1 - w.s - w.t + w.s*w.t;
	double v2w = w.s - w.s*w.t;
	double v3w = w.t - w.s*w.t;
	double v4w = w.s*w.t;
	mat << v1w, 0, v2w, 0, v3w, 0, v4w, 0,
		   0, v1w, 0, v2w, 0, v3w, 0, v4w;
	return mat;
}
BilinearWeights get_bilinear_weights(CoordinateDouble point, Coordinate upperLeftIndices, const vector<vector<CoordinateDouble>>& mesh){
	//通过mesh和左上顶点的索引计算四个顶点坐标
	CoordinateDouble p1 = mesh[upperLeftIndices.row][upperLeftIndices.col]; // topLeft
	CoordinateDouble p2 = mesh[upperLeftIndices.row][upperLeftIndices.col + 1]; // topRight
	CoordinateDouble p3 = mesh[upperLeftIndices.row + 1][upperLeftIndices.col]; // bottomLeft
	CoordinateDouble p4 = mesh[upperLeftIndices.row + 1][upperLeftIndices.col + 1]; // bottomRight

	//row为y轴，col为x轴
	double slopeTop = (p2.row - p1.row) / (p2.col - p1.col);
	double slopeBottom = (p4.row - p3.row) / (p4.col - p3.col);
	double slopeLeft = (p1.row - p3.row) / (p1.col - p3.col);
	double slopeRight = (p2.row - p4.row) / (p2.col - p4.col);

	double quadraticEpsilon = 0.01;

	//这他妈是个啥？？
	if (slopeTop == slopeBottom && slopeLeft == slopeRight) 
	{
		// method 3
		Matrix2d mat1;
		mat1 << p2.col - p1.col, p3.col - p1.col,
			    p2.row - p1.row, p3.row - p1.row;
		
		MatrixXd mat2(2,1);
		mat2 << point.col - p1.col, point.row - p1.row;

		MatrixXd matsolution = mat1.inverse()*mat2;

		BilinearWeights weights;
		weights.s = matsolution(0,0);
		weights.t = matsolution(1,0);
		return weights;
	}
	else if (slopeLeft == slopeRight) 
	{

		// method 2
		double a = (p2.col - p1.col)*(p4.row - p3.row) - (p2.row - p1.row)*(p4.col - p3.col);
		double b = point.row*((p4.col - p3.col) - (p2.col - p1.col)) - point.col*((p4.row - p3.row) - (p2.row - p1.row)) + p1.col*(p4.row - p3.row) - p1.row*(p4.col - p3.col) + (p2.col - p1.col)*(p3.row) - (p2.row - p1.row)*(p3.col);
		double c = point.row*(p3.col - p1.col) - point.col*(p3.row - p1.row) + p1.col*p3.row - p3.col*p1.row;

		double s1 = (-1 * b + sqrt(b*b - 4 * a*c)) / (2 * a);
		double s2 = (-1 * b - sqrt(b*b - 4 * a*c)) / (2 * a);
		double s;
		if (s1 >= 0 && s1 <= 1) {
			s = s1;
		}
		else if (s2 >= 0 && s2 <= 1) {
			s = s2;
		}
		else {

			if ((s1 > 1 && s1 - quadraticEpsilon < 1) ||
				(s2 > 1 && s2 - quadraticEpsilon < 1)) {
				s = 1;
			}
			else if ((s1 < 0 && s1 + quadraticEpsilon > 0) ||
				(s2 < 0 && s2 + quadraticEpsilon > 0)) {
				s = 0;
			}
			else {
				// this case should not happen
				cerr << "   Could not interpolate s weight for coordinate (" << point.col << "," << point.row << ")." << endl;
				s = 0;
			}
		}

		double val = (p3.row + (p4.row - p3.row)*s - p1.row - (p2.row - p1.row)*s);
		double t = (point.row - p1.row - (p2.row - p1.row)*s) / val;
		double valEpsilon = 0.1; // 0.1 and 0.01 appear identical
		if (fabs(val) < valEpsilon) {
			// Py ~= Cy because Dy - Cy ~= 0. So, instead of interpolating with y, we use x.
			t = (point.col - p1.col - (p2.col - p1.col)*s) / (p3.col + (p4.col - p3.col)*s - p1.col - (p2.col - p1.col)*s);
		}

		BilinearWeights weights;
		weights.s = s;
		weights.t = t;		
		return weights;
	}
	else {

		// method 1
		double a = (p3.col - p1.col)*(p4.row - p2.row) - (p3.row - p1.row)*(p4.col - p2.col);
		double b = point.row*((p4.col - p2.col) - (p3.col - p1.col)) - point.col*((p4.row - p2.row) - (p3.row - p1.row)) + (p3.col - p1.col)*(p2.row) - (p3.row - p1.row)*(p2.col) + (p1.col)*(p4.row - p2.row) - (p1.row)*(p4.col - p2.col);
		double c = point.row*(p2.col - p1.col) - (point.col)*(p2.row - p1.row) + p1.col*p2.row - p2.col*p1.row;

		double t1 = (-1 * b + sqrt(b*b - 4 * a*c)) / (2 * a);
		double t2 = (-1 * b - sqrt(b*b - 4 * a*c)) / (2 * a);
		double t;
		if (t1 >= 0 && t1 <= 1) {
			t = t1;
		}
		else if (t2 >= 0 && t2 <= 1) {
			t = t2;
		}
		else {
			if ((t1 > 1 && t1 - quadraticEpsilon < 1) ||
				(t2 > 1 && t2 - quadraticEpsilon < 1)) {
				t = 1;
			}
			else if ((t1 < 0 && t1 + quadraticEpsilon > 0) ||
				(t2 < 0 && t2 + quadraticEpsilon > 0)) {
				t = 0;
			}
			else {
				// this case should not happen
				cerr << "   Could not interpolate t weight for coordinate (" << point.col << "," << point.row << ")." << endl;
				t = 0;
			}
		}

		double val = (p2.row + (p4.row - p2.row)*t - p1.row - (p3.row - p1.row)*t);
		double s = (point.row- p1.row - (p3.row - p1.row)*t) / val;
		double valEpsilon = 0.1; // 0.1 and 0.01 appear identical
		if (fabs(val) < valEpsilon) {
			// Py ~= Ay because By - Ay ~= 0. So, instead of interpolating with y, we use x.
			s = (point.col - p1.col - (p3.col - p1.col)*t) / (p2.col + (p4.col - p2.col)*t - p1.col - (p3.col - p1.col)*t);
		}

		BilinearWeights weights;
		weights.s = clamp(s, 0, 1);
		weights.t = clamp(t, 0, 1);
		return weights;
	}
}
*/

//计算逆双线性插值计算出来的权重的对应矩阵T（2，8）维度, P = T * Vq，该矩阵用于乘以Vq=[x0,y0,...,y3,y4] V的顺序都是（左上 右上 左下 右下）
MatrixXd IBilinearWeightsToMatrix(InvBilinearWeights w)
{
	MatrixXd mat(2, 8);
	//a-左上 b-右上 d-左下 c-右下
	double a_w = 1 - w.u - w.v + w.u*w.v;
	double b_w = w.u - w.u*w.v;
	double d_w = w.v - w.u*w.v;
	double c_w = w.u*w.v;
	mat << a_w, 0, b_w, 0, d_w, 0, c_w, 0,
		   0, a_w, 0, b_w, 0, d_w, 0, c_w;
	return mat;
}

//计算a.x*b.y-a.y-b.x
double cross(CoordinateDouble a, CoordinateDouble b){ return a.col*b.row - a.row*b.col; }

//计算逆双线性插值的u,v
InvBilinearWeights get_ibilinear_weights(CoordinateDouble point, Coordinate upperLeftIndices, const vector<vector<CoordinateDouble>>& mesh)
{
	//原理 https://www.iquilezles.org/www/articles/ibilinear/ibilinear.htm
	//row为y轴，col为x轴
	//通过mesh和左上顶点的索引计算四个顶点坐标(顺序！)
	CoordinateDouble a = mesh[upperLeftIndices.row][upperLeftIndices.col]; // topLeft
	CoordinateDouble b = mesh[upperLeftIndices.row][upperLeftIndices.col + 1]; // topRight
	CoordinateDouble d = mesh[upperLeftIndices.row + 1][upperLeftIndices.col]; // bottomLeft
	CoordinateDouble c = mesh[upperLeftIndices.row + 1][upperLeftIndices.col + 1]; // bottomRight

	//E、F、G、H
	CoordinateDouble e = b - a;
	CoordinateDouble f = d - a;
	CoordinateDouble g = a - b + c - d;
	CoordinateDouble h = point - a;

	//k2,k1,k0
	double k2 = cross(g, f);
	double k1 = cross(e, f) + cross(h, g);
	double k0 = cross(h, e);

	double u, v;

	if ((int)k2 == 0)
	{
		//平行四边形
		v = -k0 / k1;
		u = (h.col - f.col*v) / (e.col + g.col*v);
	}
	else
	{
		double w = k1 * k1 - 4.0*k0*k2;
		assert(w >= 0.0);//如果point在quad内部，必然不会小于0
		w = sqrt(w);

		double v1 = (-k1 - w) / (2.0*k2);
		double u1 = (h.col - f.col*v1) / (e.col + g.col*v1);

		double v2 = (-k1 + w) / (2.0*k2);
		double u2 = (h.col - f.col*v2) / (e.col + g.col*v2);

		u = u1;
		v = v1;

		if (v<0.0 || v>1.0 || u<0.0 || u>1.0) { u = u2;   v = v2; }
		if (v-0.0 || v>1.0 || u<0.0 || u>1.0) 
		{
			//这里基本都是因为误差导致的，先忽略吧~
			//u = -1.0; 
			//v = -1.0; 
		}
	}
	return InvBilinearWeights(u, v);
}

//计算EB(V)中V的标记的矩阵（pair） V的顺序都是（左上 右上 左下 右下）
pair<SpareseMatrixD_Row, VectorXd> get_boundary_mat(const CVMat src, vector<vector<CoordinateDouble>> mesh, Config config) {
	
	int rows = config.rows;
	int cols = config.cols;
	int numMeshRow = config.meshNumRow;
	int numMeshCol = config.meshNumCol;
	int vertexnum = numMeshRow * numMeshCol;

	//顺序：[x0,y0,x1,y1...] row为y轴，col为x轴
	VectorXd dvec = VectorXd::Zero(vertexnum * 2);//dvec表标记
	VectorXd B = VectorXd::Zero(vertexnum * 2);//B表位置
	for (int i = 0; i < vertexnum * 2; i += numMeshCol * 2) {//left
		dvec(i) = 1;
		B(i) = 0;
	}//x
	for (int i = numMeshCol * 2 - 2; i < vertexnum * 2; i += numMeshCol * 2) 
	{//right
		dvec(i) = 1;
		B(i) = cols - 1;
	}//y

	for (int i = 1; i < 2 * numMeshCol; i += 2) 
	{//top
		dvec(i) = 1;
		B(i) = 0;
	}

	for (int i = 2 * vertexnum - 2 * numMeshCol + 1; i < vertexnum * 2; i += 2) 
	{//bottom
		dvec(i) = 1;
		B(i) = rows - 1;
	}

	//把dvec换成稀疏矩阵对角来存储和后续计算
	SpareseMatrixD_Row diag(dvec.size(), dvec.size());
	for (int i = 0; i < dvec.size(); i++) {
		diag.insert(i, i) = dvec(i);
	}
	diag.makeCompressed();
	return make_pair(diag, B);
};

//获取顶点坐标向量
VectorXd get_vertice(int row, int col, vector<vector<CoordinateDouble>> mesh) 
{   
	//x0,y0,x1,y1...（左上 右上 左下 右下）,row为y轴，col为x轴
	VectorXd Vq = VectorXd::Zero(8);
	CoordinateDouble p0 = mesh[row][col];//左上
	CoordinateDouble p1 = mesh[row][col + 1];//右上
	CoordinateDouble p2 = mesh[row + 1][col];//左下
	CoordinateDouble p3 = mesh[row + 1][col + 1];//右下
	Vq << p0.col, p0.row, p1.col, p1.row, p2.col, p2.row, p3.col, p3.row;
	return Vq;
}

//计算Es(V)中的存储(Aq...... - I)全部的超大矩阵 V的顺序都是（左上 右上 左下 右下）
SpareseMatrixD_Row get_shape_mat(vector<vector<CoordinateDouble>> mesh, Config config) 
{
	int numMeshRow = config.meshNumRow;
	int numMeshCol = config.meshNumCol;
	int numQuadRow = config.meshQuadRow;
	int numQuadCol = config.meshQuadRow;
	//矩阵开这么大只用了对角的子矩阵（8*8），因为是稀疏可压缩所以就没那么emmmm
	//row为y轴，col为x轴 Vq=[x0,y0,x1,y1...]
	SpareseMatrixD_Row Shape_energy(8 * numQuadRow*numQuadCol, 8 * numQuadRow*numQuadCol);
	for (int row = 0; row < numQuadRow; row++) 
	{
		for (int col = 0; col < numQuadCol; col++) 
		{
			CoordinateDouble p0 = mesh[row][col];//左上
			CoordinateDouble p1 = mesh[row][col + 1];//右上
			CoordinateDouble p2 = mesh[row + 1][col];//左下
			CoordinateDouble p3 = mesh[row + 1][col + 1];//右下
			MatrixXd Aq(8, 4);//论文中Shape Preservation处矩阵
			Aq << p0.col, -p0.row, 1, 0,
				  p0.row,  p0.col, 0, 1,
				  p1.col, -p1.row, 1, 0,
				  p1.row,  p1.col, 0, 1,
				  p2.col, -p2.row, 1, 0,
				  p2.row,  p2.col, 0, 1,
				  p3.col, -p3.row, 1, 0,
				  p3.row,  p3.col, 0, 1;

			MatrixXd Aq_trans = Aq.transpose(); //Aq^T
			MatrixXd Aq_trans_mul_Aq_reverse = (Aq_trans * Aq).inverse();//(Aq^TAq)^-1
			MatrixXd I = MatrixXd::Identity(8, 8);//单位阵
			MatrixXd coeff = (Aq*(Aq_trans_mul_Aq_reverse)*Aq_trans - I);

			int left_top_x = (row*numQuadCol + col) * 8;
			for (int i = 0; i < 8; i++) {
				for (int j = 0; j < 8; j++) {
					Shape_energy.insert(left_top_x + i, left_top_x + j) = coeff(i, j);
				}
			}
		}
	}
	Shape_energy.makeCompressed();//压缩
	return Shape_energy;
}

//计算Es(V)中的Vq标记的超大矩阵 V的顺序都是（左上 右上 左下 右下）
SpareseMatrixD_Row get_vertex_to_shape_mat(vector<vector<CoordinateDouble>> mesh, Config config) {
	int numMeshRow = config.meshNumRow;
	int numMeshCol = config.meshNumCol;
	int numQuadRow = config.meshQuadRow;
	int numQuadCol = config.meshQuadCol;
	//8指的是每个quad有8个坐标，2指的是每个顶点2个坐标；标记每个quad中的顶点，
	SpareseMatrixD_Row Q(8 * numQuadRow*numQuadCol, 2 * numMeshRow*numMeshCol);//标记Vq！
	for (int row = 0; row < numQuadRow; row++) {
		for (int col = 0; col < numQuadCol; col++) {
			int quadid = 8 * (row*numQuadCol + col);//第几个quad
			int topleftvertexId = 2 * (row*numMeshCol + col);
			Q.insert(quadid, topleftvertexId) = 1;
			Q.insert(quadid+1, topleftvertexId+1) = 1;
			Q.insert(quadid+2, topleftvertexId+2) = 1;
			Q.insert(quadid+3, topleftvertexId+3) = 1;
			Q.insert(quadid + 4, topleftvertexId + 2 * numMeshCol) = 1;
			Q.insert(quadid + 5, topleftvertexId + 2 * numMeshCol+1) = 1;
			Q.insert(quadid + 6, topleftvertexId + 2 * numMeshCol+2) = 1;
			Q.insert(quadid + 7, topleftvertexId + 2 * numMeshCol+3) = 1;
		}
	}

	Q.makeCompressed();
	return Q;
}
//暂时废了
void revise_mask_for_lines(CVMat &mask) {
	//边缘的检测不予关注
	//对mask腐蚀
	int rows = mask.rows;
	int cols = mask.cols;
	for (int row = 0; row < rows; row++) {
		mask.at<uchar>(row, 0) = 255;
		mask.at<uchar>(row, cols - 1) = 255;
	}
	for (int col = 0; col < cols; col++) {
		mask.at<uchar>(0, col) = 255;
		mask.at<uchar>(rows - 1, col) = 255;
	}
	CVMat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(8, 8));
	cv::dilate(mask, mask, element);
	cv::dilate(mask, mask, element);
}

//判断点是否在quad中，这个地方较复杂的原因是quad不一定是矩形！
bool is_in_quad(CoordinateDouble point, CoordinateDouble topLeft, CoordinateDouble topRight,
	CoordinateDouble bottomLeft, CoordinateDouble bottomRight)
{
	// the point must be to the right of the left line, below the top line, above the bottom line,
	// and to the left of the right line

	//在这里col为y轴 row为x轴 点斜式, 这个就和别的不大一样！warning...，
	//不打算改了，主要是变量名啥的不是很对应，能算就行

	// must be right of leftt line
	// 如果左上和左下处在一列的话就可以直接判断
	if (topLeft.col == bottomLeft.col)
	{
		if (point.col < topLeft.col)
			return false;
	}
	else 
	{		
		double leftSlope = (topLeft.col - bottomLeft.col) / (topLeft.row - bottomLeft.row);//k=(y2-y1)/(x2-x1)
		double yOnLineX = leftSlope * (point.row - bottomLeft.row) + bottomLeft.col;//y=k*(x-x1)+y1
		if (point.col < yOnLineX) 
			return false;
	}
	// must be left of right line
	if (topRight.col == bottomRight.col)
	{
		if (point.col > topRight.col)
			return false;
	}
	else 
	{
		double rightSlope = (topRight.col - bottomRight.col) / (topRight.row - bottomRight.row);//k=(y2-y1)/(x2-x1)
		double yOnLineX = rightSlope * (point.row - bottomRight.row) + bottomRight.col;//y=k*(x-x1)+y1
		if (point.col > yOnLineX)
			return false;	
	}
	// must be below top line
	if (topLeft.row == topRight.row) 
	{
		if (point.row < topRight.row)
			return false;
	}
	else 
	{
		double topSlope = (topRight.col - topLeft.col) / (topRight.row - topLeft.row);//k=(y2-y1)/(x2-x1)
		double xOnLineY = 1/topSlope * (point.col - topLeft.col) + topLeft.row;//x=1/k*(y-y1)+x1
		if (point.row < xOnLineY)
			return false;
	}
	// must be above bottom line
	if (bottomLeft.row == bottomRight.row) 
	{
		if (point.row > bottomRight.row)
			return false;
	}
	else 
	{

		double bottomSlope = (bottomRight.col - bottomLeft.col) / (bottomRight.row - bottomLeft.row);//k=(y2-y1)/(x2-x1)
		double xOnLineY = 1 / bottomSlope * (point.col - bottomLeft.col) + bottomLeft.row;//x=1/k*(y-y1)+x1
		if (point.row > xOnLineY) 
			return false;
	}
	// if all four constraints are satisfied, the point must be in the quad
	return true;
}

//判断这个线是否有效（沿着原图边缘是个必须处理掉的情况。。），虽然函数名不大对
bool line_in_mask(CVMat mask, LineD line)
{
	int row1 = round(line.row1), row2 = round(line.row2), 
		col1 = round(line.col1), col2 = round(line.col2);
	//不在内部不算
	if (mask.at<uchar>(row1, col1) != 0 && mask.at<uchar>(row2, col2) != 0)
		return false;
	//边界上不算
	if ((col1 == mask.cols - 1 && col2 == mask.cols - 1) ||
		(col1 == 0 && col2 == 0))
		return false;
	if ((row1 == mask.rows - 1 && row2 == mask.rows - 1) ||
		(row1 == 0 && row2 == 0))
		return false;

	//单点边界的时候
	if (row1 == 0 || row1 == mask.rows - 1 || col1 == 0 || col1 == mask.cols - 1)
	{
		try
		{
			if (mask.at<uchar>(row2 + 1, col2) == 255 || mask.at<uchar>(row2 - 1, col2) == 255
				|| mask.at<uchar>(line.row2, line.col2 + 1) == 255 || mask.at<uchar>(line.row2, line.col2 - 1) == 255)
				return false;
		}
		catch (std::exception) {}
		return true;
	}
	if (row2 == 0 || row2 == mask.rows - 1 || col2 == 0 || col2 == mask.cols - 1)
	{
		try
		{
			if (mask.at<uchar>(row1 + 1, col1) == 255 || mask.at<uchar>(row1 - 1, col1) == 255
				|| mask.at<uchar>(row1, col1 + 1) == 255 || mask.at<uchar>(row1, col1 - 1) == 255)
				return false;
		}
		catch (std::exception) {}
		return true;
	}

	//一般情况
	try
	{
		if (mask.at<uchar>(row1 + 1, col1) == 255 || mask.at<uchar>(row1 - 1, col1) == 255
			|| mask.at<uchar>(row1, col1 + 1) == 255 || mask.at<uchar>(row1, col1 - 1) == 255)
			return false;
		else
		{
			if (mask.at<uchar>(row2 + 1, col2) == 255 || mask.at<uchar>(row2 - 1, col2) == 255
				|| mask.at<uchar>(line.row2, line.col2 + 1) == 255 || mask.at<uchar>(line.row2, line.col2 - 1) == 255)
				return false;
			else return true;
		}
	}
	catch(std::exception){throw "line 的判断异常";}
}

//判断quad的某一边界与line的交点
bool does_segment_intersect_line(LineD lineSegment, double slope, double intersect, CoordinateDouble& intersectPoint)
{
	/*
    row为y轴，col为x轴，斜率k=(y2-y1)/(x2-x1) 截距b=y1-k*x1=y2-k*x2 从而y=k*x+b
	*/
	
	double lineSegmentSlope = INF;
	if (lineSegment.col1 != lineSegment.col2)
		lineSegmentSlope = (lineSegment.row2 - lineSegment.row1) / (lineSegment.col2 - lineSegment.col1);//k
	double lineSegmentIntersect = lineSegment.row1 - lineSegmentSlope * lineSegment.col1;//b

	// calculate intersection
	if (lineSegmentSlope == slope) 
	{
		//如果重叠的话！会被另外的情况下捕捉到与另外相交两边的两交点，私以为，可不管先
		if (lineSegmentIntersect == intersect) 
		{
			return false;
			// same line 该线与某个quad的边重叠  
			/*intersectPoint.col = lineSegment.col1;
			intersectPoint.row = lineSegment.row1;
			return true;*/
		}
		else return false;//平行无交点
	}

	//y=k1*x+b1 y=k2*x+b2 → 交点：x0=(b2-b1)/(k1-k2),y0=k1*x+b1
	double intersectX = (intersect - lineSegmentIntersect) / (lineSegmentSlope - slope);
	double intersectY = lineSegmentSlope * intersectX + lineSegmentIntersect;

	// 检查交点是否在线段内，用行坐标或者列坐标一个就够
	if ((intersectY <= lineSegment.row1 && intersectY >= lineSegment.row2) ||
		(intersectY <= lineSegment.row2 && intersectY >= lineSegment.row1))
	{
		intersectPoint.col = intersectX;
		intersectPoint.row = intersectY;
		return true;
	}
	else return false;

}

//当一条线的两个端点不在同一个quad中的时候，计算该线与这个quad的交点！（该quad用四个坐标表示）
vector<CoordinateDouble> intersections_with_quad(LineD lineSegment, CoordinateDouble topLeft,
	CoordinateDouble topRight, CoordinateDouble bottomLeft, CoordinateDouble bottomRight)
{
	/*
	这里可以这样理解：row为y轴，col为x轴，斜率k=(y2-y1)/(x2-x1) 截距b=y1-k*x1=y2-k*x2 从而y=k*x+b 绝了
	下述代码 **Slope=k,**Intersect=b
	*/
	vector<CoordinateDouble> intersections;

	// left
	double leftSlope = INF;
	if (topLeft.col != bottomLeft.col)
		leftSlope = (topLeft.row - bottomLeft.row) / (topLeft.col - bottomLeft.col);

	double leftIntersect = topLeft.row - leftSlope * topLeft.col;
	// check
	CoordinateDouble leftIntersectPoint;
	if (does_segment_intersect_line(lineSegment, leftSlope, leftIntersect, leftIntersectPoint))
		if (leftIntersectPoint.row >= topLeft.row && leftIntersectPoint.row <= bottomLeft.row)
			intersections.push_back(leftIntersectPoint);

	// right
	double rightSlope = INF;
	if (topRight.col != bottomRight.col)
		rightSlope = (topRight.row - bottomRight.row) / (topRight.col - bottomRight.col);
	double rightIntersect = topRight.row - rightSlope * topRight.col;
	// check
	CoordinateDouble rightIntersectPoint;
	if (does_segment_intersect_line(lineSegment, rightSlope, rightIntersect, rightIntersectPoint))
		if (rightIntersectPoint.row >= topRight.row && rightIntersectPoint.row <= bottomRight.row)
			intersections.push_back(rightIntersectPoint);

	// top
	double topSlope = INF;
	if (topLeft.col != topRight.col)
		topSlope = (topRight.row - topLeft.row) / (topRight.col - topLeft.col);
	double topIntersect = topLeft.row - topSlope * topLeft.col;
	// check
	CoordinateDouble topIntersectPoint;
	if (does_segment_intersect_line(lineSegment, topSlope, topIntersect, topIntersectPoint))
		if (topIntersectPoint.col >= topLeft.col && topIntersectPoint.col <= topRight.col)
			intersections.push_back(topIntersectPoint);

	// bottom
	double bottomSlope = INF;
	if (bottomLeft.col != bottomRight.col)
		bottomSlope = (bottomRight.row - bottomLeft.row) / (bottomRight.col - bottomLeft.col);
	double bottomIntersect = bottomLeft.row - bottomSlope * bottomLeft.col;
	// check
	CoordinateDouble bottomIntersectPoint;
	if (does_segment_intersect_line(lineSegment, bottomSlope, bottomIntersect, bottomIntersectPoint))
		if (bottomIntersectPoint.col >= bottomLeft.col && bottomIntersectPoint.col <= bottomRight.col)
			intersections.push_back(bottomIntersectPoint);

	return intersections;
}

//将每个line切割分在quad中
vector<vector<vector<LineD>>> segment_line_in_quad(const CVMat src,vector<LineD> lines, vector<vector<CoordinateDouble>> mesh,Config config) 
{
	int QuadnumRow = config.meshQuadRow;
	int QuadnumCol = config.meshQuadCol;
	vector<vector<vector<LineD>>> quad_line_seg;
	CVMat src2;
	src.copyTo(src2);
	
	for (int row = 0; row < QuadnumRow; row++) {
		vector<vector<LineD>> vec_row;
		for (int col = 0; col < QuadnumCol; col++) {
			CoordinateDouble lefttop = mesh[row][col];
			CoordinateDouble righttop = mesh[row][col+1];
			CoordinateDouble leftbottom = mesh[row+1][col];
			CoordinateDouble rightbottom = mesh[row+1][col+1];
			
			vector<LineD> lineInQuad;
			for (int i = 0; i < lines.size(); i++) 
			{
				LineD line = lines[i];
				CoordinateDouble point1(line.row1, line.col1);
				CoordinateDouble point2(line.row2, line.col2);
				bool p1InQuad = is_in_quad(point1, lefttop, righttop, leftbottom, rightbottom);
				bool p2InQuad = is_in_quad(point2, lefttop, righttop, leftbottom, rightbottom);
				if (p1InQuad && p2InQuad) 
					lineInQuad.push_back(line);
				else if (p1InQuad) 
				{
					vector<CoordinateDouble> intersections = intersections_with_quad(line, lefttop, righttop, leftbottom, rightbottom);
					//assert(intersections.size() == 1);					
					if (intersections.size() != 0) 
					{
						LineD cutLine(point1,intersections[0]);
						lineInQuad.push_back(cutLine);
					}
				}
				else if (p2InQuad) 
				{
					vector<CoordinateDouble> intersections = intersections_with_quad(line, lefttop, righttop, leftbottom, rightbottom);
					//assert(intersections.size() == 1);
					if (intersections.size() != 0) 
					{
						LineD cutLine(point2, intersections[0]);
						lineInQuad.push_back(cutLine);
					}
				}
				else 
				{
					vector<CoordinateDouble> intersections = intersections_with_quad(line, lefttop, righttop, leftbottom, rightbottom);
					if (intersections.size() ==2) {
						LineD cutLine(intersections[0], intersections[1]);
						lineInQuad.push_back(cutLine);
					}
				}
			}
			vec_row.push_back(lineInQuad);
		}
		quad_line_seg.push_back(vec_row);
	}

	return quad_line_seg;
}

//把3维的line的vector数组拉平成一维的
void flatten(vector<vector<vector<LineD>>> lineSeg, vector<LineD>& line_vec, Config config) {
	int numQuadRow = config.meshQuadRow;
	int numQuadCol = config.meshQuadCol;
	for (int row = 0; row < numQuadRow; row++) {
		for (int col = 0; col < numQuadCol; col++) {
			for (int k = 0; k < lineSeg[row][col].size(); k++) {
				line_vec.push_back(lineSeg[row][col][k]);
			}
		}
	}
}

//以对角的形式存储C*T 之后直接乘V就可以得到结果
SpareseMatrixD_Row block_diag(SpareseMatrixD_Row origin, MatrixXd addin, int QuadID, Config config) 
{
	int cols_total = 8 * config.meshQuadRow*config.meshQuadCol;
	SpareseMatrixD_Row res(origin.rows() + addin.rows(), cols_total);
	res.topRows(origin.rows()) = origin;
	
	int lefttop_row = origin.rows();
	int lefttop_col = 8 * QuadID;
	for (int row = 0; row < addin.rows(); row++) {
		for (int col = 0; col < addin.cols(); col++) {
			res.insert(lefttop_row+row, lefttop_col+col) = addin(row,col);
		}
	}
	res.makeCompressed();
	return res;
	
}

//利用lsd的代码进行line的检测
vector<LineD> lsd_detect(const CVMat src, CVMat mask)
{
	CVMat temp;
	src.copyTo(temp);

	int rows = temp.rows;
	int cols = temp.cols;
	CVMat gray_img;
	cv::cvtColor(temp, gray_img, cv::COLOR_BGR2GRAY);

	//要构造成数组用
	double *image = new double[gray_img.rows*gray_img.cols];
	for (int row = 0; row < gray_img.rows; row++)
		for (int col = 0; col < gray_img.cols; col++)
			image[row*gray_img.cols + col] = gray_img.at<uchar>(row, col);

	vector<LineD> lines;
	double * out;
	int num_lines;
	out = lsd(&num_lines, image, gray_img.cols, gray_img.rows);
	//x1, y1, x2, y2, width, p, -log_nfa. 我也不知道NFA是个啥......
	for (int i = 0; i < num_lines; i++)
	{
		LineD line(out[i * 7 + 1], out[i * 7 + 0], out[i * 7 + 3], out[i * 7 + 2]);
		if (line_in_mask(mask, line))
		{
			lines.push_back(line);
			DrawLine(temp, line);
		}
		/*cv::namedWindow("Border", cv::WINDOW_AUTOSIZE);
		cv::imshow("Border", temp);
		cv::waitKey(0);*/
	}

	/*cv::namedWindow("Border", cv::WINDOW_AUTOSIZE);
	cv::imshow("Border", temp);
	cv::waitKey(0);*/
	cv::imwrite("line_detect.png", temp);

	delete[] image;
	return lines;
}

//初始化计算三维line的vector数组（行、列、线）及线段角度
vector<vector<vector<LineD>>> init_line_seg(const CVMat src, const CVMat mask,Config config, vector < LineD > &lineSeg_flatten,
	vector<vector<CoordinateDouble>> mesh, vector<pair<int, double>>&id_theta,vector<double> &rotate_theta ) 
{
	double thetaPerbin = PI / 49;//即分成了50份从-PI/2到PI/2

	//revise_mask_for_lines(mask); 暂时作废，我觉得就不对劲

	//step1:detect line except border
	vector<LineD> lines = lsd_detect(src, mask);
	//step2: segment line in each quad
	vector<vector<vector<LineD>>> lineSeg = segment_line_in_quad(src,lines, mesh, config);
	
	//step3: flatten
	flatten(lineSeg,lineSeg_flatten, config);
	
	for (int i = 0; i < lineSeg_flatten.size(); i++) {
		LineD line = lineSeg_flatten[i];
		double theta = atan((line.row1 - line.row2) / (line.col1 - line.col2));//-PI/2~PI/2
		int lineSegmentBucket =(int) round((theta + PI / 2) / thetaPerbin);//四舍五入计算属于哪一个..0~49
		assert(lineSegmentBucket < 50);
		id_theta.push_back(make_pair(lineSegmentBucket, theta));
		rotate_theta.push_back(0);
	}
	return lineSeg;
}

//获取计算线的能量的矩阵 对角上存储（2，8）—— C*T，稀疏存储
SpareseMatrixD_Row get_line_mat(const CVMat src, CVMat mask,vector<vector<CoordinateDouble>> mesh,
	vector<double>rotate_theta, vector<vector<vector<LineD>>> lineSeg,vector<pair<MatrixXd,MatrixXd>>& BilinearVec,
	Config config,int &linenum,vector<bool>& bad) 
{
	int linetmpnum = -1;
	int rows = config.rows;
	int cols = config.cols;
	int QuadnumRow = config.meshQuadRow;
	int QuadnumCol = config.meshQuadCol;
	double gridcols = config.colPermesh;
	double gridrows = config.rowPermesh;
		
	SpareseMatrixD_Row energy_line;
	for (int row = 0; row < QuadnumRow; row++) 
	{
		for (int col = 0; col < QuadnumCol; col++) 
		{
			vector<LineD> linesegInquad = lineSeg[row][col];
			int QuadID = row * QuadnumCol + col;//就是第几个quad
			if (linesegInquad.size() == 0) {
				continue;
			}
			else {
				Coordinate topleft(row, col);//quad num的
				MatrixXd C_row_stack(0,8);
				/*if (linesegInquad.size() > 2) {
					cout << endl<<QuadID<<" "<< linesegInquad.size();
					system("pause");
				}*/
				for (int k = 0; k < linesegInquad.size(); k++) {
					linetmpnum++;
					LineD line = linesegInquad[k];
					CoordinateDouble linestart(line.row1, line.col1);
					CoordinateDouble lineend(line.row2, line.col2);
					
					//test
					InvBilinearWeights startWeight = get_ibilinear_weights(linestart, topleft, mesh);//s t2n t t1n
					MatrixXd start_W_mat = IBilinearWeightsToMatrix(startWeight);
					InvBilinearWeights endWeight = get_ibilinear_weights(lineend, topleft, mesh);
					MatrixXd end_W_mat = IBilinearWeightsToMatrix(endWeight);
					
					VectorXd S= get_vertice(row, col, mesh);
					Vector2d ans = start_W_mat * S-Vector2d(linestart.col,linestart.row);
					Vector2d ans2 = end_W_mat * S - Vector2d(lineend.col, lineend.row);
					
					//这个方案暂时不启用吧先，感觉emmm，你看不起我的逆双线性插值，虽然他有误差
					/*if (ans2.norm() >= 0.0001||ans.norm()>=0.0001) 
					{//error case
						cout << ans2.norm() << " " << ans.norm() << endl;
						bad.push_back(true);
						BilinearVec.push_back(make_pair(MatrixXd::Zero(2,8), MatrixXd::Zero(2, 8)));
						continue;
					}*/

					bad.push_back(false);

					double theta = rotate_theta[linetmpnum];	
					BilinearVec.push_back(make_pair(start_W_mat, end_W_mat));
					Matrix2d R;//旋转矩阵R
					R << cos(theta), -sin(theta), 
						 sin(theta), cos(theta);
					MatrixXd ehat(2,1);
					ehat << line.col1 - line.col2, line.row1-line.row2;//[col,row]
					MatrixXd tmp = (ehat.transpose()*ehat).inverse();
					Matrix2d I = Matrix2d::Identity();
					MatrixXd C = R * ehat*tmp*(ehat.transpose())*(R.transpose()) - I;					
					MatrixXd CT = C * (start_W_mat - end_W_mat);//CT之后乘以Vq就是了
					C_row_stack = row_stack(C_row_stack, CT);//CT (2,8)
				}
				energy_line = block_diag(energy_line, C_row_stack, QuadID,config);
			}
		}
	}
	linenum = linetmpnum;
	return energy_line;
}


//用最蠢得办法来修补。。
CVMat fill_missing_pixel(CVMat &img, const CVMat mask)
{
	//row为y轴，col为x轴
	assert(img.rows == mask.rows);
	assert(img.cols == mask.cols);
	CVMat mask_2;
	int size_erode = 9;
	CVMat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(size_erode, size_erode));
	cv::erode(mask, mask_2, element);//255是非图的部分，，腐蚀掉一些这个
	/*cv::imshow("temp", mask_2);
	cv::imshow("temp2", mask);
	cv::waitKey(0);	*/
	for (int row = 0; row < mask.rows; row++)
	{
		for (int col = 0; col < mask.cols; col++)
		{
			if (mask.at<uchar>(row, col) == 255 && mask_2.at<uchar>(row,col)==0)
			{
				for (int i = 0; i < size_erode; i++)
				{
					int temp_y = row - 2 + i / size_erode;
					int temp_x = col - 2 + i % size_erode;
					if (temp_y >= 0 && temp_y <= mask.rows&&temp_x >= 0 && temp_x <= mask.cols)
					{
						if (mask.at<uchar>(temp_y,temp_x) == 0)
						{
							img.at<cv::Vec3b>(row, col) = img.at<cv::Vec3b>(temp_y,temp_x);
							break;
						}
					}
				}
			}
		}
	}
	return mask_2;
}