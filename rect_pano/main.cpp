//https://github.com/guyuchao/rectangle-panoramic-image �����޸�

#include"Common.h"
#include"LocalWrapping.h"
#include"GlobalWrapping.h"

CVMat img;
vector<vector<CoordinateDouble>> outputmesh;
vector<vector<CoordinateDouble>> mesh;
double sx_avg = 1, sy_avg = 1;
bool flag_display = true;

GLuint matToTexture(cv::Mat mat, GLenum minFilter, GLenum magFilter, GLenum wrapFilter)
{	
	//cv::flip(mat, mat, 0);
	// Generate a number for our textureID's unique handle
	GLuint textureID;
	glGenTextures(1, &textureID);

	// Bind to our texture handle
	glBindTexture(GL_TEXTURE_2D, textureID);

	// Catch silly-mistake texture interpolation method for magnification
	if (magFilter == GL_LINEAR_MIPMAP_LINEAR ||
		magFilter == GL_LINEAR_MIPMAP_NEAREST ||
		magFilter == GL_NEAREST_MIPMAP_LINEAR ||
		magFilter == GL_NEAREST_MIPMAP_NEAREST)
	{
		//cout << "You can't use MIPMAPs for magnification - setting filter to GL_LINEAR" << endl;
		magFilter = GL_LINEAR;
	}

	// Set texture interpolation methods for minification and magnification
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, minFilter);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, magFilter);

	// Set texture clamping method
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrapFilter);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrapFilter);

	// Set incoming texture format to:
	// GL_BGR       for CV_CAP_OPENNI_BGR_IMAGE,
	// GL_LUMINANCE for CV_CAP_OPENNI_DISPARITY_MAP,
	// Work out other mappings as required ( there's a list in comments in main() )
	GLenum inputColourFormat = GL_BGR_EXT;
	if (mat.channels() == 1)
	{
		inputColourFormat = GL_LUMINANCE;
	}

	// Create the texture
	glTexImage2D(
		GL_TEXTURE_2D,     // Type of texture
		0,                 // Pyramid level (for mip-mapping) - 0 is the top level
		GL_RGB,            // Internal colour format to convert to
		mat.cols,          // Image width  i.e. 640 for Kinect in standard mode
		mat.rows,          // Image height i.e. 480 for Kinect in standard mode
		0,                 // Border width in pixels (can either be 1 or 0)
		inputColourFormat, // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
		GL_UNSIGNED_BYTE,  // Image data type
		mat.ptr());        // The actual image data itself

	// If we're using mipmaps then generate them. Note: This requires OpenGL 3.0 or higher
	//�����ʲô����
	/*if (minFilter == GL_LINEAR_MIPMAP_LINEAR ||
		minFilter == GL_LINEAR_MIPMAP_NEAREST ||
		minFilter == GL_NEAREST_MIPMAP_LINEAR ||
		minFilter == GL_NEAREST_MIPMAP_NEAREST)
	{
		glGenerateMipmap(GL_TEXTURE_2D);
	}*/

	return textureID;
}
void display() 
{	
	// �����Ļ
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	GLuint texGround = matToTexture(img);
    glViewport(0, 0, (GLsizei)img.cols, (GLsizei)img.rows);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0, img.cols, img.rows, 0);

	glEnable(GL_TEXTURE_2D);    // ��������	
	glBindTexture(GL_TEXTURE_2D, texGround);	
	if (flag_display)
	{
		for (int row = 0; row < 20; row++)
		{
			for (int col = 0; col < 20; col++)
			{
				CoordinateDouble &coord = outputmesh[row][col];
				CoordinateDouble &localcoord = mesh[row][col];
				localcoord.row /= img.rows;
				localcoord.col /= img.cols;
				//cout <<  localcoord << endl;
				//localcoord.row = clamp(localcoord.row, 0, 1);
				//localcoord.col = clamp(localcoord.col, 0, 1);
			}
		}
		flag_display = false;
	}
	for (int row = 0; row < 19; row++) {
		for (int col = 0; col < 19; col++) {
			CoordinateDouble local_left_top = mesh[row][col];
			CoordinateDouble local_right_top = mesh[row][col + 1];
			CoordinateDouble local_left_bottom = mesh[row + 1][col];
			CoordinateDouble local_right_bottom = mesh[row + 1][col + 1];

			CoordinateDouble global_left_top = outputmesh[row][col];
			CoordinateDouble global_right_top = outputmesh[row][col + 1];
			CoordinateDouble global_left_bottom = outputmesh[row + 1][col];
			CoordinateDouble global_right_bottom = outputmesh[row + 1][col + 1];
			
			glBegin(GL_QUADS);
			glTexCoord2d(local_left_top.col, local_left_top.row); glVertex2d(global_left_top.col, global_left_top.row);
			glTexCoord2d(local_right_top.col, local_right_top.row); glVertex2d(global_right_top.col,  global_right_top.row);
			glTexCoord2d(local_right_bottom.col, local_right_bottom.row); glVertex2d(global_right_bottom.col,  global_right_bottom.row);
			glTexCoord2d(local_left_bottom.col, local_left_bottom.row);	glVertex2d(global_left_bottom.col,  global_left_bottom.row);		
			glEnd();			
		}
	}
	glDisable(GL_TEXTURE_2D);
	glutSwapBuffers();
}

int main(int argc, char* argv[]) 
{
	//�����������ǲ������������ʾ�ģ������bug�Ҿ����ˣ����
	//img = cv::imread("C:\\Users\\lz183\\Desktop\\������\\�Ͽ�\\����\\rect_pano\\rect_pano\\data\\5_input.jpg");
	//cv::resize(img, img, cv::Size(0, 0), 0.20, 0.20);
		
    img = cv::imread("C:\\Users\\lz183\\Desktop\\������\\�Ͽ�\\����\\rect_pano\\rect_pano\\data\\1_input.jpg");
	cv::resize(img, img, cv::Size(0, 0), 0.22, 0.22);

	//img = cv::imread("C:\\Users\\lz183\\Desktop\\������\\�Ͽ�\\����\\rect_pano\\rect_pano\\data\\2.png");

	//img = cv::imread("C:\\Users\\lz183\\Desktop\\������\\�Ͽ�\\����\\rect_pano\\rect_pano\\data\\1.jpg");
	
	//seam carving��Ч�������

	double Time = (double)cv::getTickCount();
	double fator_scale = 1.0;
	
	CVMat scaled_img;
	cv::resize(img, scaled_img, cv::Size(0, 0), 1.0 / fator_scale, 1.0 / fator_scale);
	Config config(img.rows, img.cols, 20, 20);//����������400������
	CVMat mask = Mask_contour(scaled_img);
	CVMat tmpmask;
	mask.copyTo(tmpmask);
	CVMat warpped_img = CVMat::zeros(scaled_img.size(), CV_8UC3);

	//�����local warp��Ҫ��ʱ�任��improve�汾��....����
	//scaled_img��ԭͼ��warpped_img�Ǿ��ε�
	vector<vector<Coordinate>> displacementMap = Local_wrap(scaled_img, warpped_img, tmpmask);
	mesh = get_rectangle_mesh(config);
	warp_mesh_back(mesh, displacementMap, config);//mesh��λ�ƻ�ԭͼ��

	cout << "mesh warp back done~" << endl;
	double temp_time = (double)cv::getTickCount() - Time;
	std::printf("run time = %f s\n", temp_time / (cv::getTickFrequency()));

	/*���漸����ϡ�����Ĵ洢��ʽ����׼��˽��Ϊ��Ϊ�˺���������С���õ�*/

	//�洢(Aq...... - I)ȫ���ĳ������(���Խ��ϵ�8*8�Ӿ���)
	SpareseMatrixD_Row shape_energy = get_shape_mat(mesh, config);
	
	//Vq��ǵĳ������
	SpareseMatrixD_Row Q = get_vertex_to_shape_mat(mesh, config);
	cout << "prepare shape energy done~" << endl;

	//B�ж�V�ı�ǵľ��󣨶ԣ�
	pair<SpareseMatrixD_Row, VectorXd> pair_dvec_B = get_boundary_mat(scaled_img, mesh, config);
	cout << "get border constraint done~" << endl;

	vector<pair<int, double>>id_theta;//������theta��quantize����M=50��
	vector < LineD > line_flatten;
	vector<double> rotate_theta;
	vector<vector<vector<LineD>>> LineSeg = init_line_seg(scaled_img, mask, config, line_flatten, mesh, id_theta, rotate_theta);
	vector<pair<MatrixXd, MatrixXd>> BilinearVec_origin;
	//10 iteration
	for (int iter = 1; iter <= 10; iter++) 
	{
		cout << iter << endl;
		int Nl = 0;
		vector<pair<MatrixXd, MatrixXd>> BilinearVec;//need to update�Ҳ���Ϊ��Ӧ�ø��°�
		vector<bool> bad;
		//������Ҳ��һ������ľ��󣬶Խ��ϴ洢��2��8������ C*T��ϡ��洢
		SpareseMatrixD_Row line_energy = get_line_mat(scaled_img, mask, mesh, rotate_theta, LineSeg, BilinearVec, config, Nl, bad);
		cout << "get line energy, number of lines " << Nl << endl;
		if (iter == 1)BilinearVec_origin = BilinearVec;//��� �������

		//combine
		double Nq = config.meshQuadRow*config.meshQuadCol;//quad number
		double lambdaB = 10e8;
		double lambdaL = 100;


		//��������κ�������С��ʵ����ת��������С���˵ĸ���(b-K2*x)'(b-K2*x)����С,
		//��x_hat=(K2'K2)^-1K2'b ��л��Ԫͳ�Ʒ���

		//����ط���ϵ���ǲ�����������Ҹо��ǣ��ѿ��ˣ������ϸȷ����
		SpareseMatrixD_Row shape = (1 / sqrt(Nq))*(shape_energy*Q);
		SpareseMatrixD_Row boundary = sqrt(lambdaB) * pair_dvec_B.first;
		SpareseMatrixD_Row line = sqrt((lambdaL / Nl))*(line_energy*Q);

		SpareseMatrixD_Row K = row_stack(shape, line);
		SpareseMatrixD_Row K2 = row_stack(K, boundary);
		VectorXd B = pair_dvec_B.second;
		VectorXd b = VectorXd::Zero(K2.rows());
		b.tail(B.size()) = sqrt(lambdaB) * B;//�����жѵ��󣬺���B.size()��ֵ�Ͳ�����0
		SparseMatrixD K2_trans = K2.transpose();
		MatrixXd temp = K2_trans * K2;
		temp = temp.inverse();
		VectorXd x;	
		x = temp * K2_trans*b;

	    //update theta
		outputmesh = vector_to_mesh(x, config);
		int tmplinenum = -1;
		VectorXd thetagroup = VectorXd::Zero(50);
		VectorXd thetagroupcnt = VectorXd::Zero(50);
		for (int row = 0; row < config.meshQuadRow; row++) 
		{
			for (int col = 0; col < config.meshQuadCol; col++)
			{
				vector<LineD> linesegInquad = LineSeg[row][col];
				int QuadID = row * config.meshQuadCol + col;
				if (linesegInquad.size() == 0) 
					continue;
				else {
					VectorXd S = get_vertice(row, col, outputmesh);
					for (int k = 0; k < linesegInquad.size(); k++) {
						tmplinenum++;
						if (bad[tmplinenum] == true)
							continue;//������ʱ��ȡ����
						pair<MatrixXd, MatrixXd> Bstartend = BilinearVec_origin[tmplinenum];					
						MatrixXd start_W_mat = Bstartend.first;
						MatrixXd end_W_mat = Bstartend.second;
						Vector2d newstart = start_W_mat * S;
						Vector2d newend = end_W_mat * S;

						double theta = atan((newstart(1) - newend(1)) / (newstart(0) - newend(0)));
						double deltatheta = theta - id_theta[tmplinenum].second;
						if (isnan(id_theta[tmplinenum].second) || isnan(deltatheta)) 
							continue;
						if (deltatheta > (PI / 2))
							deltatheta -= PI;					
						if (deltatheta < (-PI / 2))
							deltatheta += PI;
						thetagroup(id_theta[tmplinenum].first) += deltatheta;
						thetagroupcnt(id_theta[tmplinenum].first) += 1;
					}
				}
			}
		}
		//cal mean theta;��ÿ��bin����ƽ����ת��
		for (int i = 0; i < thetagroup.size(); i++)
			thetagroup(i) /= thetagroupcnt(i);
		//update rotate_theta
		for (int i = 0; i < rotate_theta.size(); i++)
			rotate_theta[i] = thetagroup[id_theta[i].first];		
	}

	enlarge_mesh(mesh, fator_scale, fator_scale, config);//��Ϊ��ʼ��С��
	enlarge_mesh(outputmesh, fator_scale, fator_scale, config);

	CVMat erode_mask;
	draw_savemesh(img,"quad1.png", mesh, config);//���棬ע�͵��˻�ͼ�Ĵ���
	draw_savemesh(img,"quad2.png", outputmesh, config);
	//Ϊʲô�޲���Ч����ô���
	//Ҫ�Ӹ�������Ҿ��ã���Ҫ��mask�жϵ�ʱ�򲢲�����ȫ׼ȷ�ģ�������������ɣ��ټ� 2020.8.9
	erode_mask=fill_missing_pixel(img, mask);//�޲�����
	erode_mask=fill_missing_pixel(img, erode_mask);//�޲�����
	//erode_mask=fill_missing_pixel(img, erode_mask);//�޲�����
	
	compute_scaling(sx_avg, sy_avg, mesh, outputmesh, config);//������������
    enlarge_mesh(mesh, 1/sx_avg, 1/sy_avg, config);
	enlarge_mesh(outputmesh, 1 / sx_avg, 1 / sy_avg, config);
	cv::resize(img, img, cv::Size(0, 0), 1 / sx_avg, 1 / sy_avg);
	draw_savemesh(img, "quad1_scaling.png", mesh, config);//���棬ע�͵��˻�ͼ�Ĵ���
	draw_savemesh(img, "quad2_scaling.png", outputmesh, config);

	temp_time = (double)cv::getTickCount() - Time;
	std::printf("run time = %f s\n", temp_time / (cv::getTickFrequency()));
	//glut
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);	
	glutInitWindowSize(img.cols, img.rows);
	glutInitWindowPosition(100, 100);	
	glutCreateWindow("Panoramic_image");
	glEnable(GL_DEPTH_TEST);
	glutDisplayFunc(&display);   //ע�ắ�� 

	temp_time = (double)cv::getTickCount() - Time;
	std::printf("run time = %f s\n", temp_time / (cv::getTickFrequency()));
	glutMainLoop();
	
	return 0;
}