#include"LocalWrapping.h"

//��sort�õıȽϺ���
bool cmp(const pair<int, float> a, const pair<int, float> b) {
	return a.second<b.second;
}

//ͨ��mask���ж��Ƿ�͸���������ж��Ƿ�ȱʧ�ɣ�Ѱ��seam carving����ͼ�õ��ж�
bool Is_transparent(CVMat mask, int row, int col) 
{
	if (mask.at<uchar>(row, col) == 0) return false;//0����ͼ������
	else return true;
}

//��ʼ��
void init_displacement(vector<vector<Coordinate>>& displacement, int rows, int cols) {
	for (int row = 0; row < rows; row++) {
		vector<Coordinate> displacement_row;
		for (int col = 0; col < cols; col++) {
			Coordinate c;
			displacement_row.push_back(c);
		}
		displacement.push_back(displacement_row);
	}
}

//��sobel�����ڻҶ�ͼ���ϼ����ݶ�
//https://github.com/axu2/improved-seam-carving 
//�о���������һ������ļ��㷽��Ҳ���ԣ�����C++��������Ҳ̫����/(��o��)/~~
CVMat Sobel_img(CVMat src) {
	CVMat gray;
	cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
	CVMat grad_x, grad_y, dst;
	cv::Sobel(src, grad_x, CV_32F, 1, 0, 3);
	cv::Sobel(src, grad_y, CV_32F, 0, 1, 3);
	//addWeighted(grad_x, 0.5, grad_y, 0.5, 0, dst);
	dst = cv::abs(grad_x) + cv::abs(grad_y);
	return dst;
}

//��λ�ƾ���Ѿ���mesh�ƻ�ȥ
void warp_mesh_back(vector<vector<CoordinateDouble>>& mesh, vector<vector<Coordinate>> displacementMap, Config config) {
	int meshnum_row = config.meshNumRow;
	int meshnum_col = config.meshNumCol;
	//floor ������x���������
	for (int row_mesh = 0; row_mesh < meshnum_row; row_mesh++) {
		for (int col_mesh = 0; col_mesh < meshnum_col; col_mesh++) {		
			CoordinateDouble& meshVertexCoord = mesh[row_mesh][col_mesh];
			Coordinate vertexDisplacement = displacementMap[(int)floor(meshVertexCoord.row)][(int)floor(meshVertexCoord.col)];
			meshVertexCoord.row += vertexDisplacement.row;
			meshVertexCoord.col += vertexDisplacement.col;
		}
	}
}

//�ҵ�������ı߽�
pair<int, int> Choose_longest_border(CVMat src, CVMat mask, Border& direction) 
{
	//�����border��[begin��end]
	int maxLength = 0;
	int rows = src.rows;
	int cols = src.cols;

	int final_startIndex = 0;//startIndex����һ����ʼֱ��endIndex���� 
	int final_endIndex = 0;

	int tmp_maxLength, tmp_startIndex, tmp_endIndex;
	
	//left	
	tmp_maxLength = tmp_startIndex = tmp_endIndex = 0;
	bool isCounting = false;
	for (int row = 0; row < rows; row++) 
	{
		if (!Is_transparent(mask, row, 0) || row == rows - 1) 
		{
			if (isCounting) 
			{//������ڼ���������Ϊֹ������֮ǰ���
				if (Is_transparent(mask, row, 0)) //����ж���Ϊ���ж����һ��Ԫ�ص�
				{
					tmp_endIndex++;
					tmp_maxLength++;
					isCounting = true;
				}
				if (tmp_maxLength > maxLength) {
					maxLength = tmp_maxLength;
					final_startIndex = tmp_startIndex;
					final_endIndex = tmp_endIndex;
					direction = BORDER_LEFT;
				}
			}
			isCounting = false;
			tmp_startIndex = tmp_endIndex = row + 1;//Ӧ���Ǵ���һ����ʼ��
			tmp_maxLength = 0;
		}
		else {//�õ�͸������ʼ����
			tmp_endIndex++;
			tmp_maxLength++;
			isCounting = true;
		}
	}
	//cout << maxLength << endl;
	//right
	tmp_maxLength = tmp_startIndex = tmp_endIndex = 0;
	isCounting = false;
	for (int row = 0; row < rows; row++) 
	{
		if (!Is_transparent(mask, row, cols - 1) || row == rows - 1) 
		{
			if (isCounting) {//������ڼ���������Ϊֹ������֮ǰ���
				if (Is_transparent(mask, row, cols - 1)) 
				{
					tmp_endIndex++;
					tmp_maxLength++;
					isCounting = true;
				}
				if (tmp_maxLength > maxLength) 
				{
					maxLength = tmp_maxLength;
					final_startIndex = tmp_startIndex;
					final_endIndex = tmp_endIndex;
					direction = BORDER_RIGHT;
				}
			}
			isCounting = false;
			tmp_startIndex = tmp_endIndex = row + 1;
			tmp_maxLength = 0;
		}
		else {//�õ�͸������ʼ����
			tmp_endIndex++;
			tmp_maxLength++;
			isCounting = true;
		}
	}
	//cout << maxLength << endl;
	//top
	tmp_maxLength = tmp_startIndex = tmp_endIndex = 0;
	isCounting = false;
	for (int col = 0; col < cols; col++) {
		//cout << col << " " << Is_transparent(point) << endl;
		if (!Is_transparent(mask, 0, col) || col == cols - 1) {
			if (isCounting) {//������ڼ���������Ϊֹ������֮ǰ���
				if (Is_transparent(mask, 0, col)) {
					tmp_endIndex++;
					tmp_maxLength++;
					isCounting = true;
				}
				if (tmp_maxLength > maxLength) {
					maxLength = tmp_maxLength;
					final_startIndex = tmp_startIndex;
					final_endIndex = tmp_endIndex;
					direction = BORDER_TOP;
				}
			}
			isCounting = false;
			tmp_startIndex = tmp_endIndex = col + 1;
			tmp_maxLength = 0;
		}
		else {//�õ�͸������ʼ����
			tmp_endIndex++;
			tmp_maxLength++;
			isCounting = true;
		}
	}
	//cout << maxLength << endl;
	//bottom
	tmp_maxLength = tmp_startIndex = tmp_endIndex = 0;
	isCounting = false;
	for (int col = 0; col < cols; col++) {
		if (!Is_transparent(mask, rows - 1, col) || col == cols - 1) {
			if (isCounting) {//������ڼ���������Ϊֹ������֮ǰ���
				if (Is_transparent(mask, rows - 1, col)) {
					tmp_endIndex++;
					tmp_maxLength++;
					isCounting = true;
				}
				if (tmp_maxLength > maxLength) {
					maxLength = tmp_maxLength;
					final_startIndex = tmp_startIndex;
					final_endIndex = tmp_endIndex;
					direction = BORDER_BOTTOM;
				}
			}
			isCounting = false;
			tmp_startIndex = tmp_endIndex = col + 1;
			tmp_maxLength = 0;
		}
		else {//�õ�͸������ʼ����
			tmp_endIndex++;
			tmp_maxLength++;
			isCounting = true;
		}
	}
	//cout << maxLength << endl;
	if(maxLength==0)return make_pair(0, 0);
	return make_pair(final_startIndex, final_endIndex - 1);
}

//���ǰѱ߽续��������
CVMat Show_longest_border(CVMat src, pair<int, int>begin_end, Border direction) {
	CVMat tmpsrc;
	src.copyTo(tmpsrc);
	int rows = src.rows;
	int cols = src.cols;
	switch (direction) {
	case BORDER_LEFT:
		for (int row = begin_end.first; row < begin_end.second; row++)
			tmpsrc.at<colorPixel>(row, 0) = colorPixel(0, 0, 255);
		break;
	case BORDER_RIGHT:
		for (int row = begin_end.first; row < begin_end.second; row++)
			tmpsrc.at<colorPixel>(row, cols - 1) = colorPixel(0, 0, 255);
		break;
	case BORDER_TOP:
		for (int col = begin_end.first; col < begin_end.second; col++)
			tmpsrc.at<colorPixel>(0, col) = colorPixel(0, 0, 255);
		break;
	case BORDER_BOTTOM:
		for (int col = begin_end.first; col < begin_end.second; col++)
			tmpsrc.at<colorPixel>(rows - 1, col) = colorPixel(0, 0, 255);
		break;
	default:
		break;
	}

	cv::namedWindow("Border", cv::WINDOW_AUTOSIZE);
	cv::imshow("Border", tmpsrc);
	cv::waitKey(0);

	return tmpsrc;
}

//ִ��local warp
vector<vector<Coordinate>> Local_wrap(const CVMat src, CVMat& warp_img, CVMat mask) 
{
	src.copyTo(warp_img);
	//���warp_img���ɾ���
	vector<vector<Coordinate>> displacementMap = Get_Local_warp_displacement(warp_img, mask);
	return displacementMap;
}

//ͨ��xxx���������ջ��local warp��λ�ƾ���,����ͨ�����ý�warp_img��ɾ���
vector<vector<Coordinate>> Get_Local_warp_displacement(CVMat& warp_img, CVMat mask) {

	CVMat seam_img;//����������ͼ����
	warp_img.copyTo(seam_img);

	int rows = warp_img.rows;
	int cols = warp_img.cols;

	vector<vector<Coordinate>> displacementMap;//��������洢����λ�ƾ���
	vector<vector<Coordinate>> finaldisplacementMap;//���������ʱ����
	init_displacement(finaldisplacementMap, rows, cols);//��ʼ���Ķ���0
	init_displacement(displacementMap, rows, cols);
	//int cnt=0;

	while (true) {
		Border direction;
		pair<int, int> begin_end = Choose_longest_border(warp_img, mask, direction);//ÿ�ζ�ѡ����ı߽�
		//cout << direction<<endl;
		//Show_longest_border(src, begin_end, direction); ���������������һ�����ͼƬ
		if (begin_end.first == begin_end.second) 
		{
			//cv::imwrite("local_warping.png", warp_img);//������֮�󱣴���һ���Ľ��
			cv::imwrite("seam_img.png", seam_img);
			return displacementMap;
		}
		else 
		{
			bool shift_to_end = false;//�����ں��������б�־�������ң����ϻ����£�������
			//���ݱ߽�ѡ��seam caring�ķ���ˮƽ����ֱ��
			SeamDirection seamdirection;
			switch (direction) 
			{
			case BORDER_LEFT:
				seamdirection = SEAM_VERTICAL;
				shift_to_end = false;
				break;
			case BORDER_RIGHT:
				seamdirection = SEAM_VERTICAL;
				shift_to_end = true;
				break;
			case BORDER_TOP:
				seamdirection = SEAM_HORIZONTAL;
				shift_to_end = false;
				break;
			case BORDER_BOTTOM:
				seamdirection = SEAM_HORIZONTAL;
				shift_to_end = true;
				break;
			default:
				throw "directionֵ�쳣";
				break;
			}

			int* seam = Get_local_seam_improved(warp_img, mask, seamdirection, begin_end);
			//int* seam = Get_local_seam(warp_img, mask, seamdirection, begin_end);
		
			warp_img = Insert_local_seam(warp_img, seam_img,mask, seam, seamdirection, begin_end, shift_to_end);

			//����λ�ƾ���
			for (int row = 0; row < rows; row++) 
			{
				for (int col = 0; col < cols; col++) 
				{
					Coordinate tmpdisplacement;
					if (seamdirection == SEAM_VERTICAL && row >= begin_end.first&&row <= begin_end.second) 
					{
						int local_row = row - begin_end.first;
						if (shift_to_end)
						{
							if (col > seam[local_row])
								tmpdisplacement.col = -1;
						}
						else
						{
							if (col < seam[local_row])
								tmpdisplacement.col = 1;
						}
					}
					else 
					{
						if (seamdirection == SEAM_HORIZONTAL && col >= begin_end.first&&col <= begin_end.second) 
						{
							int local_col = col - begin_end.first;
							if (shift_to_end)
							{
								if (row > seam[local_col])
									tmpdisplacement.row = -1;
							}
							else
							{
								if (row < seam[local_col])
									tmpdisplacement.row = 1;
							}
						}
					}
					Coordinate &finaldisplacement = finaldisplacementMap[row][col];
					int tmpdisplace_row = row + tmpdisplacement.row;
					int tmpdisplace_col = col + tmpdisplacement.col;
					Coordinate displacementOftarget = displacementMap[tmpdisplace_row][tmpdisplace_col];
					int rowInOrigin = tmpdisplace_row + displacementOftarget.row;
					int colInOrigin = tmpdisplace_col + displacementOftarget.col;
					finaldisplacement.row = rowInOrigin - row;
					finaldisplacement.col = colInOrigin - col;

				}
			}
			for (int row = 0; row < rows; row++) {
				for (int col = 0; col < cols; col++) {
					Coordinate &displacement = displacementMap[row][col];
					Coordinate finalDisplacement = finaldisplacementMap[row][col];
					displacement.row = finalDisplacement.row;
					displacement.col = finalDisplacement.col;
				}
			}

		}
	}
	throw "seam carving �쳣��ֹ";
}

//��seamִ�в������
CVMat Insert_local_seam(CVMat src, CVMat& seam_img,CVMat& mask, int* seam, SeamDirection seamdirection, pair<int, int> begin_end, bool shiftToend) 
{
	//���輴ת��
	if (seamdirection == SEAM_HORIZONTAL) 
	{//seamҲ�Ǹ������ת�ù���
		cv::transpose(src, src);
		cv::transpose(seam_img, seam_img);
		cv::transpose(mask, mask);
	}

	CVMat resimg;
	src.copyTo(resimg);

	int begin = begin_end.first;//�border���ڵ�local row��Χ
	int end = begin_end.second;

	int rows = src.rows;
	int cols = src.cols;

	for (int row = begin; row <= end; row++) {
		int local_row = row - begin;		
		if (!shiftToend) //�����˼������������»����ң���������
			for (int col = 0; col < seam[local_row]; col++)
			{
				resimg.at<colorPixel>(row, col) = src.at<colorPixel>(row, col + 1);
				seam_img.at<colorPixel>(row, col) = seam_img.at<colorPixel>(row, col + 1);
				mask.at<uchar>(row, col) = mask.at<uchar>(row, col + 1);
			}
		else
			for (int col = cols - 1; col > seam[local_row]; col--)
			{
				resimg.at<colorPixel>(row, col) = src.at<colorPixel>(row, col - 1);
				seam_img.at<colorPixel>(row, col) = seam_img.at<colorPixel>(row, col - 1);
				mask.at<uchar>(row, col) = mask.at<uchar>(row, col - 1);
			}
		
		//��seam�ϵ�ֵ�����ˣ�����ƽ�������߽������������
		mask.at<uchar>(row, seam[local_row]) = 0;
		if (seam[local_row] == 0);
			//resimg.at<colorPixel>(row, seam[local_row]) = src.at<colorPixel>(row, seam[local_row] + 1);
		else
		{
			if (seam[local_row] == cols - 1);
				//resimg.at<colorPixel>(row, seam[local_row]) = src.at<colorPixel>(row, seam[local_row] - 1);
			else 
			{
				colorPixel pixel1 = src.at<colorPixel>(row, seam[local_row] + 1);
				colorPixel pixel2 = src.at<colorPixel>(row, seam[local_row] - 1);
				resimg.at<colorPixel>(row, seam[local_row]) = 0.5*pixel1 + 0.5*pixel2;
				
			}
		}
		seam_img.at<colorPixel>(row, seam[local_row]) = colorPixel(0, 255, 0);
	}

	if (seamdirection == SEAM_HORIZONTAL)
	{//�����ת�ù���Ҫת�û���
		cv::transpose(resimg, resimg);
		cv::transpose(seam_img, seam_img);
		cv::transpose(mask, mask);
	}
	
	/*cv::namedWindow("insert_seam", cv::WINDOW_AUTOSIZE);
	cv::imshow("insert_seam", resimg);
	cv::waitKey(0);*/

	return resimg;
}

//����seam carving
int* Get_local_seam(CVMat src, CVMat mask, SeamDirection seamdirection, pair<int, int> begin_end) 
{
	//ˮƽ����ֻ�轫ͼƬת��..�����ֻ�账��ֱ
	if (seamdirection == SEAM_HORIZONTAL)
	{
		cv::transpose(src, src);
		cv::transpose(mask, mask);
	}

	//ͳһѰ����ֱ��seam
	int rows = src.rows;
	int cols = src.cols;

	int row_start = begin_end.first;
	int row_end = begin_end.second;

	int range = row_end - row_start + 1;

	int col_start = 0;
	int col_end = cols - 1;

	int outputWidth = cols;
	int outputHeight = range;

	CVMat displayimg;
	src.copyTo(displayimg);

	CVMat local_img = displayimg(cv::Range::Range(row_start, row_end + 1), cv::Range::Range(col_start, col_end + 1));
	CVMat local_mask = mask(cv::Range::Range(row_start, row_end + 1), cv::Range::Range(col_start, col_end + 1));//Range ����ҿ�

	CVMat local_energy = Sobel_img(local_img);

	//��ͼ�в���������Ϊ�����
	for (int row = 0; row < range; row++)
		for (int col = col_start; col <= col_end; col++)
			if ((int)local_mask.at<uchar>(row, col) == 255) local_energy.at<float>(row, col) = INF;

	//DP������Сseam
	CVMat tmpenergy;//���ָ����seam���ۼ�����
	local_energy.copyTo(tmpenergy);
	for (int row = 1; row < range; row++) {
		for (int col = col_start; col <= col_end; col++)
		{
			if ((int)local_mask.at<uchar>(row, col) == 255)
				continue;
			if (col == col_start)
				tmpenergy.at<float>(row, col) += min(tmpenergy.at<float>(row - 1, col), tmpenergy.at<float>(row - 1, col + 1));
			else 
			{
				if (col == col_end)
					tmpenergy.at<float>(row, col) += min(tmpenergy.at<float>(row - 1, col - 1), tmpenergy.at<float>(row - 1, col));
				else
					tmpenergy.at<float>(row, col) += min(tmpenergy.at<float>(row - 1, col), min(tmpenergy.at<float>(row - 1, col - 1), tmpenergy.at<float>(row - 1, col + 1)));			
			}
		}
	}

	//����Сseam����
	vector<pair<int, float>> last_row;
	for (int col = col_start; col <= col_end; col++)
		last_row.push_back(make_pair(col, tmpenergy.at<float>(range - 1, col)));

	sort(last_row.begin(), last_row.end(), cmp);
	int *seam = new int[range];
	seam[range - 1] = last_row[0].first;

	//����
	for (int row = range - 2; row >= 0; row--) {
		if (seam[row + 1] == col_start) 
		{
			if (tmpenergy.at<float>(row, seam[row + 1] + 1) < tmpenergy.at<float>(row, seam[row + 1]))
				seam[row] = seam[row + 1] + 1;
			else seam[row] = seam[row + 1];
		}
		else {
			if (seam[row + 1] == col_end) 
			{
				if (tmpenergy.at<float>(row, seam[row + 1] - 1) < tmpenergy.at<float>(row, seam[row + 1]))
					seam[row] = seam[row + 1] - 1;
				else
					seam[row] = seam[row + 1];
			}
			else 
			{
				float min_energy = min(tmpenergy.at<float>(row, seam[row + 1] - 1), min(tmpenergy.at<float>(row, seam[row + 1]), tmpenergy.at<float>(row, seam[row + 1] + 1)));
				if (min_energy == tmpenergy.at<float>(row, seam[row + 1] - 1))
					seam[row] = seam[row + 1] - 1;
				else 
				{
					if (min_energy == tmpenergy.at<float>(row, seam[row + 1] + 1))
						seam[row] = seam[row + 1] + 1;
					else seam[row] = seam[row + 1];
				}
			}
		}
	}

	/*for (int row = 0; row < range; row++)
		local_img.at<colorPixel>(row, seam[row]) = colorPixel(255, 0, 0);

	cv::namedWindow("seam", cv::WINDOW_AUTOSIZE);
	cv::imshow("seam", local_img);
	cv::waitKey(0);*/

	return seam;//����ˮƽ������������Ҫת�û�����������̻�ת���Զ�Ӧ
}

//����improved seam carving  https://github.com/axu2/improved-seam-carving
int* Get_local_seam_improved(CVMat src, CVMat mask, SeamDirection seamdirection, pair<int, int> begin_end)
{
	//ˮƽ����ֻ�轫ͼƬת��..�����ֻ�账��ֱ
	if (seamdirection == SEAM_HORIZONTAL)
	{
		cv::transpose(src, src);
		cv::transpose(mask, mask);
	}
	//ͳһѰ����ֱ��seam
	int rows = src.rows;
	int cols = src.cols;

	int row_start = begin_end.first;
	int row_end = begin_end.second;

	int range = row_end - row_start + 1;

	int col_start = 0;
	int col_end = cols - 1;

	int outputWidth = cols;
	int outputHeight = range;

	CVMat displayimg;
	src.copyTo(displayimg);

	CVMat local_img = displayimg(cv::Range::Range(row_start, row_end + 1), cv::Range::Range(col_start, col_end + 1));
	CVMat local_mask = mask(cv::Range::Range(row_start, row_end + 1), cv::Range::Range(col_start, col_end + 1));//Range ����ҿ�

	CVMat gray;
	cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);


	CVMat U = (cv::Mat_<int>(3, 3) << 0, 0, 0, -1, 0, 1, 0, 0, 0);
	CVMat L_ = (cv::Mat_<int>(3, 3) << 0, 1, 0, -1, 0, 0, 0, 0, 0);
	CVMat R_ = (cv::Mat_<int>(3, 3) << 0, 1, 0, 0, 0, -1, 0, 0, 0);
	CVMat cU, cL, cR;
	filter2D(gray, cU, CV_64F, U);
	filter2D(gray, cL, CV_64F, U);
	filter2D(gray, cR, CV_64F, U);
	cU = abs(cU);
	cL = abs(cL) + cU;
	cR = abs(cR) + cU;
	CVMat M(range, cols, CV_64FC1, cv::Scalar(0));

	//��ͼ�в���������Ϊ�����
	for (int row = 0; row < range; row++)
		for (int col = col_start; col <= col_end; col++)
			if ((int)local_mask.at<uchar>(row, col) == 255) M.at<double>(row, col) = INF;

	for (int i = 1; i < range; i++)
		for (int j = col_start; j <= col_end; j++)
		{
			double cu_temp = cU.at<double>(i, j);
			double cl_temp = cL.at<double>(i, j);
			double cr_temp = cR.at<double>(i, j);
			double MU = M.at<double>(i - 1, j) + cu_temp;
			double ML = M.at<double>(i - 1, j - 1) + cl_temp;
			double MR = M.at<double>(i - 1, j + 1) + cr_temp;

			if ((int)local_mask.at<uchar>(i, j) == 255)
				continue;
			M.at<double>(i, j) = min(MU, min(ML, MR));
			//cout << min(cu_temp, min(cl_temp, cr_temp)) << endl;
		}

	

	//DP������Сseam
	CVMat tmpenergy;//���ָ����seam���ۼ�����
	M.copyTo(tmpenergy);

	//����Сseam����
	vector<pair<int, double>> last_row;
	for (int col = col_start; col <= col_end; col++)
		last_row.push_back(make_pair(col, tmpenergy.at<double>(range - 1, col)));
	sort(last_row.begin(), last_row.end(), cmp);
	int *seam = new int[range];
	seam[range - 1] = last_row[0].first;

	//����
	for (int row = range - 2; row >= 0; row--) 
	{
		if (seam[row + 1] == col_start)
		{
			if (tmpenergy.at<double>(row, seam[row + 1] + 1) < tmpenergy.at<double>(row, seam[row + 1]))
				seam[row] = seam[row + 1] + 1;
			else seam[row] = seam[row + 1];
		}
		else {
			if (seam[row + 1] == col_end)
			{
				if (tmpenergy.at<double>(row, seam[row + 1] - 1) < tmpenergy.at<double>(row, seam[row + 1]))
					seam[row] = seam[row + 1] - 1;
				else
					seam[row] = seam[row + 1];
			}
			else
			{
				double min_energy = min(tmpenergy.at<double>(row, seam[row + 1] - 1), min(tmpenergy.at<double>(row, seam[row + 1]), tmpenergy.at<double>(row, seam[row + 1] + 1)));
				if (min_energy == tmpenergy.at<double>(row, seam[row + 1] - 1))
					seam[row] = seam[row + 1] - 1;
				else
				{
					if (min_energy == tmpenergy.at<double>(row, seam[row + 1] + 1))
						seam[row] = seam[row + 1] + 1;
					else seam[row] = seam[row + 1];
				}
			}
		}
	}

	for (int row = 0; row < range; row++)
		local_img.at<colorPixel>(row, seam[row]) = colorPixel(255, 0, 0);

	/*cv::namedWindow("seam", cv::WINDOW_AUTOSIZE);
	cv::imshow("seam", local_img);
	cv::waitKey(0);*/

	return seam;//����ˮƽ������������Ҫת�û�����������̻�ת���Զ�Ӧ
}

//����mesh�������꣨0~row-1��0~col-1�Ķ
vector<vector<CoordinateDouble>> get_rectangle_mesh(Config config) {
	int rows = config.rows;
	int cols = config.cols;
	int meshnum_row = config.meshNumRow;
	int meshnum_col = config.meshNumCol;
	double row_per_mesh = config.rowPermesh;
	double col_per_mesh = config.colPermesh;
	vector<vector<CoordinateDouble>> mesh;
	for (int row_mesh = 0; row_mesh < meshnum_row; row_mesh++) {
		vector<CoordinateDouble> meshrow;
		for (int col_mesh = 0; col_mesh < meshnum_col; col_mesh++) {
			CoordinateDouble coord;
			coord.row = row_mesh * row_per_mesh;
			coord.col = col_mesh * col_per_mesh;
			meshrow.push_back(coord);
		}
		mesh.push_back(meshrow);
	}
	return mesh;
}