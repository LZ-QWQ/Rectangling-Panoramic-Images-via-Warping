#include"GlobalWrapping.h"

//ע�����������������Ҳ�����զ�������� �����ϰɡ��� �Ӿ����쿴�������˫���Բ�ֵ��һ���ģ�ֻ�����ͺ����զ��ġ���
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
	//ͨ��mesh�����϶�������������ĸ���������
	CoordinateDouble p1 = mesh[upperLeftIndices.row][upperLeftIndices.col]; // topLeft
	CoordinateDouble p2 = mesh[upperLeftIndices.row][upperLeftIndices.col + 1]; // topRight
	CoordinateDouble p3 = mesh[upperLeftIndices.row + 1][upperLeftIndices.col]; // bottomLeft
	CoordinateDouble p4 = mesh[upperLeftIndices.row + 1][upperLeftIndices.col + 1]; // bottomRight

	//rowΪy�ᣬcolΪx��
	double slopeTop = (p2.row - p1.row) / (p2.col - p1.col);
	double slopeBottom = (p4.row - p3.row) / (p4.col - p3.col);
	double slopeLeft = (p1.row - p3.row) / (p1.col - p3.col);
	double slopeRight = (p2.row - p4.row) / (p2.col - p4.col);

	double quadraticEpsilon = 0.01;

	//�������Ǹ�ɶ����
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

//������˫���Բ�ֵ���������Ȩ�صĶ�Ӧ����T��2��8��ά��, P = T * Vq���þ������ڳ���Vq=[x0,y0,...,y3,y4] V��˳���ǣ����� ���� ���� ���£�
MatrixXd IBilinearWeightsToMatrix(InvBilinearWeights w)
{
	MatrixXd mat(2, 8);
	//a-���� b-���� d-���� c-����
	double a_w = 1 - w.u - w.v + w.u*w.v;
	double b_w = w.u - w.u*w.v;
	double d_w = w.v - w.u*w.v;
	double c_w = w.u*w.v;
	mat << a_w, 0, b_w, 0, d_w, 0, c_w, 0,
		   0, a_w, 0, b_w, 0, d_w, 0, c_w;
	return mat;
}

//����a.x*b.y-a.y-b.x
double cross(CoordinateDouble a, CoordinateDouble b){ return a.col*b.row - a.row*b.col; }

//������˫���Բ�ֵ��u,v
InvBilinearWeights get_ibilinear_weights(CoordinateDouble point, Coordinate upperLeftIndices, const vector<vector<CoordinateDouble>>& mesh)
{
	//ԭ�� https://www.iquilezles.org/www/articles/ibilinear/ibilinear.htm
	//rowΪy�ᣬcolΪx��
	//ͨ��mesh�����϶�������������ĸ���������(˳��)
	CoordinateDouble a = mesh[upperLeftIndices.row][upperLeftIndices.col]; // topLeft
	CoordinateDouble b = mesh[upperLeftIndices.row][upperLeftIndices.col + 1]; // topRight
	CoordinateDouble d = mesh[upperLeftIndices.row + 1][upperLeftIndices.col]; // bottomLeft
	CoordinateDouble c = mesh[upperLeftIndices.row + 1][upperLeftIndices.col + 1]; // bottomRight

	//E��F��G��H
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
		//ƽ���ı���
		v = -k0 / k1;
		u = (h.col - f.col*v) / (e.col + g.col*v);
	}
	else
	{
		double w = k1 * k1 - 4.0*k0*k2;
		assert(w >= 0.0);//���point��quad�ڲ�����Ȼ����С��0
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
			//�������������Ϊ���µģ��Ⱥ��԰�~
			//u = -1.0; 
			//v = -1.0; 
		}
	}
	return InvBilinearWeights(u, v);
}

//����EB(V)��V�ı�ǵľ���pair�� V��˳���ǣ����� ���� ���� ���£�
pair<SpareseMatrixD_Row, VectorXd> get_boundary_mat(const CVMat src, vector<vector<CoordinateDouble>> mesh, Config config) {
	
	int rows = config.rows;
	int cols = config.cols;
	int numMeshRow = config.meshNumRow;
	int numMeshCol = config.meshNumCol;
	int vertexnum = numMeshRow * numMeshCol;

	//˳��[x0,y0,x1,y1...] rowΪy�ᣬcolΪx��
	VectorXd dvec = VectorXd::Zero(vertexnum * 2);//dvec����
	VectorXd B = VectorXd::Zero(vertexnum * 2);//B��λ��
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

	//��dvec����ϡ�����Խ����洢�ͺ�������
	SpareseMatrixD_Row diag(dvec.size(), dvec.size());
	for (int i = 0; i < dvec.size(); i++) {
		diag.insert(i, i) = dvec(i);
	}
	diag.makeCompressed();
	return make_pair(diag, B);
};

//��ȡ������������
VectorXd get_vertice(int row, int col, vector<vector<CoordinateDouble>> mesh) 
{   
	//x0,y0,x1,y1...������ ���� ���� ���£�,rowΪy�ᣬcolΪx��
	VectorXd Vq = VectorXd::Zero(8);
	CoordinateDouble p0 = mesh[row][col];//����
	CoordinateDouble p1 = mesh[row][col + 1];//����
	CoordinateDouble p2 = mesh[row + 1][col];//����
	CoordinateDouble p3 = mesh[row + 1][col + 1];//����
	Vq << p0.col, p0.row, p1.col, p1.row, p2.col, p2.row, p3.col, p3.row;
	return Vq;
}

//����Es(V)�еĴ洢(Aq...... - I)ȫ���ĳ������ V��˳���ǣ����� ���� ���� ���£�
SpareseMatrixD_Row get_shape_mat(vector<vector<CoordinateDouble>> mesh, Config config) 
{
	int numMeshRow = config.meshNumRow;
	int numMeshCol = config.meshNumCol;
	int numQuadRow = config.meshQuadRow;
	int numQuadCol = config.meshQuadRow;
	//������ô��ֻ���˶Խǵ��Ӿ���8*8������Ϊ��ϡ���ѹ�����Ծ�û��ôemmmm
	//rowΪy�ᣬcolΪx�� Vq=[x0,y0,x1,y1...]
	SpareseMatrixD_Row Shape_energy(8 * numQuadRow*numQuadCol, 8 * numQuadRow*numQuadCol);
	for (int row = 0; row < numQuadRow; row++) 
	{
		for (int col = 0; col < numQuadCol; col++) 
		{
			CoordinateDouble p0 = mesh[row][col];//����
			CoordinateDouble p1 = mesh[row][col + 1];//����
			CoordinateDouble p2 = mesh[row + 1][col];//����
			CoordinateDouble p3 = mesh[row + 1][col + 1];//����
			MatrixXd Aq(8, 4);//������Shape Preservation������
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
			MatrixXd I = MatrixXd::Identity(8, 8);//��λ��
			MatrixXd coeff = (Aq*(Aq_trans_mul_Aq_reverse)*Aq_trans - I);

			int left_top_x = (row*numQuadCol + col) * 8;
			for (int i = 0; i < 8; i++) {
				for (int j = 0; j < 8; j++) {
					Shape_energy.insert(left_top_x + i, left_top_x + j) = coeff(i, j);
				}
			}
		}
	}
	Shape_energy.makeCompressed();//ѹ��
	return Shape_energy;
}

//����Es(V)�е�Vq��ǵĳ������ V��˳���ǣ����� ���� ���� ���£�
SpareseMatrixD_Row get_vertex_to_shape_mat(vector<vector<CoordinateDouble>> mesh, Config config) {
	int numMeshRow = config.meshNumRow;
	int numMeshCol = config.meshNumCol;
	int numQuadRow = config.meshQuadRow;
	int numQuadCol = config.meshQuadCol;
	//8ָ����ÿ��quad��8�����꣬2ָ����ÿ������2�����ꣻ���ÿ��quad�еĶ��㣬
	SpareseMatrixD_Row Q(8 * numQuadRow*numQuadCol, 2 * numMeshRow*numMeshCol);//���Vq��
	for (int row = 0; row < numQuadRow; row++) {
		for (int col = 0; col < numQuadCol; col++) {
			int quadid = 8 * (row*numQuadCol + col);//�ڼ���quad
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
//��ʱ����
void revise_mask_for_lines(CVMat &mask) {
	//��Ե�ļ�ⲻ���ע
	//��mask��ʴ
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

//�жϵ��Ƿ���quad�У�����ط��ϸ��ӵ�ԭ����quad��һ���Ǿ��Σ�
bool is_in_quad(CoordinateDouble point, CoordinateDouble topLeft, CoordinateDouble topRight,
	CoordinateDouble bottomLeft, CoordinateDouble bottomRight)
{
	// the point must be to the right of the left line, below the top line, above the bottom line,
	// and to the left of the right line

	//������colΪy�� rowΪx�� ��бʽ, ����ͺͱ�Ĳ���һ����warning...��
	//��������ˣ���Ҫ�Ǳ�����ɶ�Ĳ��Ǻܶ�Ӧ���������

	// must be right of leftt line
	// ������Ϻ����´���һ�еĻ��Ϳ���ֱ���ж�
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

//�ж�������Ƿ���Ч������ԭͼ��Ե�Ǹ����봦������������������Ȼ�����������
bool line_in_mask(CVMat mask, LineD line)
{
	int row1 = round(line.row1), row2 = round(line.row2), 
		col1 = round(line.col1), col2 = round(line.col2);
	//�����ڲ�����
	if (mask.at<uchar>(row1, col1) != 0 && mask.at<uchar>(row2, col2) != 0)
		return false;
	//�߽��ϲ���
	if ((col1 == mask.cols - 1 && col2 == mask.cols - 1) ||
		(col1 == 0 && col2 == 0))
		return false;
	if ((row1 == mask.rows - 1 && row2 == mask.rows - 1) ||
		(row1 == 0 && row2 == 0))
		return false;

	//����߽��ʱ��
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

	//һ�����
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
	catch(std::exception){throw "line ���ж��쳣";}
}

//�ж�quad��ĳһ�߽���line�Ľ���
bool does_segment_intersect_line(LineD lineSegment, double slope, double intersect, CoordinateDouble& intersectPoint)
{
	/*
    rowΪy�ᣬcolΪx�ᣬб��k=(y2-y1)/(x2-x1) �ؾ�b=y1-k*x1=y2-k*x2 �Ӷ�y=k*x+b
	*/
	
	double lineSegmentSlope = INF;
	if (lineSegment.col1 != lineSegment.col2)
		lineSegmentSlope = (lineSegment.row2 - lineSegment.row1) / (lineSegment.col2 - lineSegment.col1);//k
	double lineSegmentIntersect = lineSegment.row1 - lineSegmentSlope * lineSegment.col1;//b

	// calculate intersection
	if (lineSegmentSlope == slope) 
	{
		//����ص��Ļ����ᱻ���������²�׽���������ཻ���ߵ������㣬˽��Ϊ���ɲ�����
		if (lineSegmentIntersect == intersect) 
		{
			return false;
			// same line ������ĳ��quad�ı��ص�  
			/*intersectPoint.col = lineSegment.col1;
			intersectPoint.row = lineSegment.row1;
			return true;*/
		}
		else return false;//ƽ���޽���
	}

	//y=k1*x+b1 y=k2*x+b2 �� ���㣺x0=(b2-b1)/(k1-k2),y0=k1*x+b1
	double intersectX = (intersect - lineSegmentIntersect) / (lineSegmentSlope - slope);
	double intersectY = lineSegmentSlope * intersectX + lineSegmentIntersect;

	// ��齻���Ƿ����߶��ڣ������������������һ���͹�
	if ((intersectY <= lineSegment.row1 && intersectY >= lineSegment.row2) ||
		(intersectY <= lineSegment.row2 && intersectY >= lineSegment.row1))
	{
		intersectPoint.col = intersectX;
		intersectPoint.row = intersectY;
		return true;
	}
	else return false;

}

//��һ���ߵ������˵㲻��ͬһ��quad�е�ʱ�򣬼�����������quad�Ľ��㣡����quad���ĸ������ʾ��
vector<CoordinateDouble> intersections_with_quad(LineD lineSegment, CoordinateDouble topLeft,
	CoordinateDouble topRight, CoordinateDouble bottomLeft, CoordinateDouble bottomRight)
{
	/*
	�������������⣺rowΪy�ᣬcolΪx�ᣬб��k=(y2-y1)/(x2-x1) �ؾ�b=y1-k*x1=y2-k*x2 �Ӷ�y=k*x+b ����
	�������� **Slope=k,**Intersect=b
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

//��ÿ��line�и����quad��
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

//��3ά��line��vector������ƽ��һά��
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

//�ԶԽǵ���ʽ�洢C*T ֮��ֱ�ӳ�V�Ϳ��Եõ����
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

//����lsd�Ĵ������line�ļ��
vector<LineD> lsd_detect(const CVMat src, CVMat mask)
{
	CVMat temp;
	src.copyTo(temp);

	int rows = temp.rows;
	int cols = temp.cols;
	CVMat gray_img;
	cv::cvtColor(temp, gray_img, cv::COLOR_BGR2GRAY);

	//Ҫ�����������
	double *image = new double[gray_img.rows*gray_img.cols];
	for (int row = 0; row < gray_img.rows; row++)
		for (int col = 0; col < gray_img.cols; col++)
			image[row*gray_img.cols + col] = gray_img.at<uchar>(row, col);

	vector<LineD> lines;
	double * out;
	int num_lines;
	out = lsd(&num_lines, image, gray_img.cols, gray_img.rows);
	//x1, y1, x2, y2, width, p, -log_nfa. ��Ҳ��֪��NFA�Ǹ�ɶ......
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

//��ʼ��������άline��vector���飨�С��С��ߣ����߶νǶ�
vector<vector<vector<LineD>>> init_line_seg(const CVMat src, const CVMat mask,Config config, vector < LineD > &lineSeg_flatten,
	vector<vector<CoordinateDouble>> mesh, vector<pair<int, double>>&id_theta,vector<double> &rotate_theta ) 
{
	double thetaPerbin = PI / 49;//���ֳ���50�ݴ�-PI/2��PI/2

	//revise_mask_for_lines(mask); ��ʱ���ϣ��Ҿ��þͲ��Ծ�

	//step1:detect line except border
	vector<LineD> lines = lsd_detect(src, mask);
	//step2: segment line in each quad
	vector<vector<vector<LineD>>> lineSeg = segment_line_in_quad(src,lines, mesh, config);
	
	//step3: flatten
	flatten(lineSeg,lineSeg_flatten, config);
	
	for (int i = 0; i < lineSeg_flatten.size(); i++) {
		LineD line = lineSeg_flatten[i];
		double theta = atan((line.row1 - line.row2) / (line.col1 - line.col2));//-PI/2~PI/2
		int lineSegmentBucket =(int) round((theta + PI / 2) / thetaPerbin);//�����������������һ��..0~49
		assert(lineSegmentBucket < 50);
		id_theta.push_back(make_pair(lineSegmentBucket, theta));
		rotate_theta.push_back(0);
	}
	return lineSeg;
}

//��ȡ�����ߵ������ľ��� �Խ��ϴ洢��2��8������ C*T��ϡ��洢
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
			int QuadID = row * QuadnumCol + col;//���ǵڼ���quad
			if (linesegInquad.size() == 0) {
				continue;
			}
			else {
				Coordinate topleft(row, col);//quad num��
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
					
					//���������ʱ�����ð��ȣ��о�emmm���㿴�����ҵ���˫���Բ�ֵ����Ȼ�������
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
					Matrix2d R;//��ת����R
					R << cos(theta), -sin(theta), 
						 sin(theta), cos(theta);
					MatrixXd ehat(2,1);
					ehat << line.col1 - line.col2, line.row1-line.row2;//[col,row]
					MatrixXd tmp = (ehat.transpose()*ehat).inverse();
					Matrix2d I = Matrix2d::Identity();
					MatrixXd C = R * ehat*tmp*(ehat.transpose())*(R.transpose()) - I;					
					MatrixXd CT = C * (start_W_mat - end_W_mat);//CT֮�����Vq������
					C_row_stack = row_stack(C_row_stack, CT);//CT (2,8)
				}
				energy_line = block_diag(energy_line, C_row_stack, QuadID,config);
			}
		}
	}
	linenum = linetmpnum;
	return energy_line;
}


//������ð취���޲�����
CVMat fill_missing_pixel(CVMat &img, const CVMat mask)
{
	//rowΪy�ᣬcolΪx��
	assert(img.rows == mask.rows);
	assert(img.cols == mask.cols);
	CVMat mask_2;
	int size_erode = 9;
	CVMat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(size_erode, size_erode));
	cv::erode(mask, mask_2, element);//255�Ƿ�ͼ�Ĳ��֣�����ʴ��һЩ���
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