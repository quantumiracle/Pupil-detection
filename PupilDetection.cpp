
#include "PupilDetection.h"

#define GRAY_DIFF_THRESHOLD 60		//�Ҷ�ֵ��ֵ��ֵ�����ڸ�ֵ����Ϊ�������

#define SRCIMG_X_EXTENT 10			//ԭͼ��x�ĵ�����Χ
#define SRCIMG_Y_EXTENT 10			//ԭͼ��y�ĵ�����Χ
#define SRCIMG_A_EXTENT 10			//ԭͼ��a�ĵ�����Χ
#define SRCIMG_B_EXTENT 10			//ԭͼ��b�ĵ�����Χ
#define SRCIMG_ALPHA_EXTENT 0.5		//ԭͼ��alpha�ĵ�����Χ
#define SRCIMG_FRAME_THRESHOLD 20	//ԭͼ�߿��С

#define G_L_THRESHOLD 215	//ͼ��������ֵ
#define CONTOURS_MIN_SIZE 5	//���Բ�ܵ�����ٸ���
#define CONTOURS_MAX_SIZE 100	//���Բ�ܵ��������
#define G_L_OFFSET 4		//������G_L_OFFSETֵ��Ϊ��������
#define G_L_DIFF_TH 20		//������ʱ��4�������ʱ��ֵ����ֵ

#define IMG_MIN_POINT 15	//Сͼ��Բ������ٸ�����С�ڸ�ֵ�򲻿���
#define SRCIMG_MIN_POINT 50	//ԭͼ��Բ������ٸ�����С�ڸ�ֵ�򲻿���

#define OUT_OFFSET 3
#define IN_OFFSET 3

void PDDrawRect(Mat &img, Point pt1, Point pt2);

PupilDetection::PupilDetection(Mat &_img, int _offset)
{
	_img.copyTo(m_img);
	m_offset = _offset;

	//�õ�theta����
	int thetaLen = TWOPI / THETA_INCREMENT;
	int k = 0;
	m_cosTheta = new double [thetaLen];
	m_sinTheta = new double [thetaLen];
	for (k = 0; k < thetaLen; k ++)
	{
		m_cosTheta[k] = cos(THETA_INCREMENT * k);
		m_sinTheta[k] = sin(THETA_INCREMENT * k);
	}

	//�õ�alpha����
	int alphaLen = PD_PI / ALPHA_INCREMENT;
	m_cosAlpha = new double [alphaLen + 1];
	m_sinAlpha = new double [alphaLen + 1];
	for (k = 0; k < alphaLen + 1; k ++)
	{
		m_cosAlpha[k] = cos(ALPHA_INCREMENT * k);
		m_sinAlpha[k] = sin(ALPHA_INCREMENT * k);
	}

}

PupilDetection::PupilDetection()
{

	//�õ�theta����
	int thetaLen = TWOPI / THETA_INCREMENT;
	int k = 0;
	m_cosTheta = new double [thetaLen];
	m_sinTheta = new double [thetaLen];
	for (k = 0; k < thetaLen; k ++)
	{
		m_cosTheta[k] = cos(THETA_INCREMENT * k);
		m_sinTheta[k] = sin(THETA_INCREMENT * k);
	}

	//�õ�alpha����
	int alphaLen = PD_PI / ALPHA_INCREMENT;
	m_cosAlpha = new double [alphaLen + 1];
	m_sinAlpha = new double [alphaLen + 1];
	for (k = 0; k < alphaLen + 1; k ++)
	{
		m_cosAlpha[k] = cos(ALPHA_INCREMENT * k);
		m_sinAlpha[k] = sin(ALPHA_INCREMENT * k);
	}
}

PupilDetection::~PupilDetection()
{

	if (NULL != m_cosTheta)
	{
		delete m_cosTheta;
		m_cosTheta = NULL;
	}
	if (NULL != m_sinTheta)
	{
		delete m_sinTheta;
		m_sinTheta = NULL;
	}
	if (NULL != m_cosAlpha)
	{
		delete m_cosAlpha;
		m_cosAlpha = NULL;
	}
	if (NULL != m_sinAlpha)
	{
		delete m_sinAlpha;
		m_sinAlpha = NULL;
	}
}

/* ����ͼ�� */
int PupilDetection::setImg(Mat &_img)
{
	_img.copyTo(m_img);
	return SUCCEED;

}

/* ����ellipse */
void PupilDetection::getEllipse(PDEllipse &_ellipse)
{
	_ellipse.x = m_ellipse.x;
	_ellipse.y = m_ellipse.y;
	_ellipse.a = m_ellipse.a;
	_ellipse.b = m_ellipse.b;
	_ellipse.alpha = m_ellipse.alpha;

	return ;
}


/* ����ͼ���ϸ�����ݶ�ֵ */
void PupilDetection::GradCalcu(void)
{

	int imgElement = 0;
	int imgLeftElement = 0;
	int imgRightElement = 0;
	int imgTopElement = 0;
	int imgBottomElement = 0;
	int leftVal, topVal = 0;
	const double srcimg_img_ratio = (double)IMGWIDTH / SRCIMGWIDTH;
	const int MIN_X = max(int(m_minX * srcimg_img_ratio + m_offset), FRAME_THRESHOLD);
	const int MIN_Y = max(int(m_minY * srcimg_img_ratio + m_offset), FRAME_THRESHOLD);
	const int MAX_X = min(int(m_maxX * srcimg_img_ratio - m_offset), IMGWIDTH - FRAME_THRESHOLD);
	const int MAX_Y = min(int(m_maxY * srcimg_img_ratio - m_offset), IMGWIDTH - FRAME_THRESHOLD);
/*
	for (int x = FRAME_THRESHOLD + m_offset; x < MAX_X; x ++)
	{
		for (int y = FRAME_THRESHOLD + m_offset; y < MAX_Y; y ++)
		{
		*/
	for (int x = MIN_X; x < MAX_X; x ++)
	{
		for (int y = MIN_Y; y < MAX_Y; y ++)
		{
			imgElement = m_img.data[y * IMGWIDTH + x];
			imgLeftElement = m_img.data[y * IMGWIDTH + x - m_offset];
			imgTopElement = m_img.data[(y - m_offset) * IMGWIDTH + x];
			//ȥ����ߵ�
/*			if (imgElement > GLINT_THRESHOLD || imgLeftElement > GLINT_THRESHOLD || imgTopElement > GLINT_THRESHOLD)	
			{
				m_grads[y][x] = 0;
				continue;
			}
*/
			leftVal = abs(imgLeftElement - imgElement);
			topVal = abs(imgTopElement - imgElement);
			//ȥ����ֵ���󣬼������Χ�����
//			if (leftVal > GRAY_DIFF_THRESHOLD || topVal > GRAY_DIFF_THRESHOLD)m_grads[y][x] = -1;
//			else m_grads[y][x] = leftVal + topVal;
			m_grads[y * IMGWIDTH + x] = leftVal + topVal;

		}
	}
}


/* ���������Բ��ƫ�����µĲ�ּ��� */
double PupilDetection::EllipDiffCalcu(const int _x, const int _y, const int count)
{
	double val = EllipseCalcu(_x, _y, count);

	return val;
}

/* ������Բ�߽����ص��ݶ�ƽ��ֵ */
double PupilDetection::EllipseCalcu(const int _x, const int _y, const int count)
{
	int valueAccum = 0;

	//������ĻҶ�ֵ����һ��ֵ�����ų��������
	if ((m_img.data[_y * IMGWIDTH + _x]) > CENTER_THRESHOLD)
	{
		return -1;
	}
	
	int x = 0, y = 0;
	const int increment = _y * IMGWIDTH + _x;
	int tempPointVal;
//	int maxK = vecPoints.size();
//	const int count = vecPoints.size();

	for (int k = 0; k < count; k ++)
	{
		//��Բ�ϵ�����ļ���
//		tempPointVal = vecPoints[k];
		tempPointVal = m_peripheryPoints[k] + increment;

		//�ݶ�����
		valueAccum += m_grads[tempPointVal];

	}//for theta

	return (double)valueAccum / count;
}

/* ����ͫ�׼�ⶨλ����⵽����Բ����ellipse�� */
int PupilDetection::DetectPupil(void)
{

	if (NULL == m_img.data)
	{
#if _DEBUG
		cout << "ͼ��Ϊ�գ�" << endl;
#endif
		return FAILED;
	}
	//FRAME_THRESHOLD >= offset,��ֹ���ݶ�����ʱx - offset��y - offset����ͼ����
	if (m_offset < 1 || FRAME_THRESHOLD < m_offset)
	{
#if _DEBUG
		cout << "ƫ����m_offset < 1 or FRAME_THRESHOLD < m_offset��" << endl;
#endif
		return FAILED;
	}
	//����ͼ��
	if (FAILED == ImgProcessing())
	{
#if _DEBUG
		cout << "ͼ����ʧ�ܣ�" << endl;
#endif
		return FAILED;
	}
	
	//����ͼ���ϸ���4��������ݶ�ֵ
	GradCalcu();

	double maxVal = 0;
	double tempVal = 0;
	vector<int> vecPoints;
	
	int minX, minY, maxX, maxY = 0;
	int tempX = -1;
	int tempPointCount, maxPointCount = 0;
//	const int MAX_A = min(IMGHEIGHT, IMGWIDTH) / 2 - m_offset;
	const int MAX_A = min(m_maxX - m_minX, m_maxY - m_minY) / 2 - m_offset;
	const double srcimg_img_ratio = (double)IMGWIDTH / SRCIMGWIDTH;
	const int MIN_X = max(int(m_minX * srcimg_img_ratio), FRAME_THRESHOLD);
	const int MIN_Y = max(int(m_minY * srcimg_img_ratio), FRAME_THRESHOLD);
	const int MAX_X = min(int(m_maxX * srcimg_img_ratio), IMGWIDTH - FRAME_THRESHOLD);
	const int MAX_Y = min(int(m_maxY * srcimg_img_ratio), IMGWIDTH - FRAME_THRESHOLD);
	const int MAX_ALPHA_K = PD_PI / ALPHA_INCREMENT + 1;
	if (MAX_A <= m_offset)
	{
		return FAILED;
	}


	for (int a = m_offset + 1; a < MAX_A; a ++)
	{
		for (int b = max(A_B_THRESHOLD, int(a * RATIO_THRESHOLD)); b <= a ;b ++) 
		{

			for (int alphaK = 0; alphaK < MAX_ALPHA_K; alphaK ++)
			{
				//�������a, b, alpha�µ���ԲԲ�ܸ���
				tempPointCount = PeripheryPointCalcu(a, b, alphaK, minX, maxX, minY, maxY);
				if (tempPointCount < 0)
				{
					continue;
				}
//				if (tempPointCount > maxPointCount)maxPointCount = tempPointCount;
/*				minX = FRAME_THRESHOLD - minX + m_offset;
				maxX = IMGWIDTH - FRAME_THRESHOLD - maxX - m_offset;
				minY = FRAME_THRESHOLD - minY + m_offset;
				maxY = IMGHEIGHT - FRAME_THRESHOLD - maxY - m_offset;
*/
				minX = MIN_X - minX + m_offset;
				maxX = MAX_X - maxX - m_offset;
				minY = MIN_Y - minY + m_offset;
				maxY = MAX_Y - maxY - m_offset;
				for (int x = minX; x < maxX; x ++)
				{
					for (int y = minY; y < maxY; y ++)
					{
						tempVal = EllipDiffCalcu(x, y, tempPointCount);

						if (maxVal < tempVal)
						{
							maxVal = tempVal;
							m_ellipse.x = x;
							m_ellipse.y = y;
							m_ellipse.a = a;
							m_ellipse.b = b;
							m_ellipse.alpha = ALPHA_INCREMENT * alphaK;
						}//if
						
					}//y

				}//x
				if (a == b)		//a == b��������һ��Բ��alphaΪ0�͹���
				{
					break;
				}//if
			}//alphaK
		}//b
	}//a

//	cout << "maxPointCount: " << maxPointCount << endl;

	//�滭��Բ
//	PDDrawEllipse(m_img, IMGWIDTH, IMGHEIGHT);

	//ԭͼ�������
	SrcImgDetectPupil();

	return SUCCEED;
}
/* �������a, b, alpha�µ���ԲԲ�ܸ��� */
int PupilDetection::PeripheryPointCalcu(const int a, const int b, const int alphaK,
										 int &minX, int &maxX, int &minY, int &maxY)
{
//	vecPoints.clear();

	int x = -1, y = -1;
	double d_x = -1, d_y = -1;
	int oldX = -1, oldY = -1;
	int tempPointVal;
	int count = 0;

	minX = 0;
	maxX = 0;
	minY = 0;
	maxY = 0;

	const double COS_OUT_B = m_cosAlpha[alphaK] * b;
	const double COS_OUT_A = m_cosAlpha[alphaK] * a;
	const double SIN_OUT_B = m_sinAlpha[alphaK] * b;
	const double SIN_OUT_A = m_sinAlpha[alphaK] * a;

	const int MAX_K = TWOPI / THETA_INCREMENT;

	for (int k = 0; k < MAX_K; k ++)
	{
		//����Բ�ϵ�����ļ���
		d_x = COS_OUT_B * m_cosTheta[k] - SIN_OUT_A * m_sinTheta[k];
		d_y = SIN_OUT_B * m_cosTheta[k] + COS_OUT_A * m_sinTheta[k];

		x = cvRound(d_x);
		y = cvRound(d_y);
/*
		x = int(d_x + 0.5);
		y = int(d_y + 0.5);
*/
		//ֻͳ���µ�����
		if (x != oldX || y != oldY)
		{
		
			//��
			if (x < 0)
			{
				if (x < minX)minX = x;						
			}
			//��
			else 
			{
				if (x > maxX)maxX = x;
			}

			//��
			if (y < 0)
			{
				if (y < minY)minY = y;						
			}
			//��
			else 
			{
				if (y > maxY)maxY = y;
			}											
			
//			tempPointVal = y * IMGWIDTH + x;
			m_peripheryPoints[count] = y * IMGWIDTH + x;
			count ++;
//			vecPoints.push_back(tempPointVal);
			oldX = x;
			oldY = y;
		}
	
		
	}

//	const int count = vecPoints.size();
	if (count < IMG_MIN_POINT)return -1;

	return count;
}

/* ����ͼ�� */
int PupilDetection::getImg(Mat &_img)
{
	m_srcImg.copyTo(_img);
//	m_img.copyTo(_img);

	return SUCCEED;
}

/* ������Բ */
void PupilDetection::PDDrawEllipse(Mat &_img, int imgWidth, int imgHeight)
{
	int oldX = 0;
	int oldY = 0;
	int count = 0;
	double valueAccum = 0;

	if (m_ellipse.alpha < 0)
	{
		m_ellipse.alpha += PD_PI;
	}

	int centerX = m_ellipse.x;
	int centerY = m_ellipse.y;
	int a = m_ellipse.a;
	int b = m_ellipse.b;
	int alphaK = m_ellipse.alpha / ALPHA_INCREMENT;

	int x = -1, y = -1;
	double d_x = -1, d_y = -1;
	const double COS_B = m_cosAlpha[alphaK] * b;
	const double COS_A = m_cosAlpha[alphaK] * a;
	const double SIN_B = m_sinAlpha[alphaK] * b;
	const double SIN_A = m_sinAlpha[alphaK] * a;
	const int MAX_K = TWOPI / THETA_INCREMENT;

	//����
	_img.data[centerY * imgWidth + centerX] = 255;
	_img.data[(centerY + 1) * imgWidth + centerX] = 255;
	_img.data[(centerY - 1) * imgWidth + centerX] = 255;
	_img.data[centerY * imgWidth + centerX + 1] = 255;
	_img.data[centerY * imgWidth + centerX - 1] = 255;

	for (int k = 0; k < MAX_K; k ++)
	{
		//��Բ�ϵ�����ļ���
		d_x = COS_B * m_cosTheta[k] - SIN_A * m_sinTheta[k] + centerX;
		d_y = SIN_B * m_cosTheta[k] + COS_A * m_sinTheta[k] + centerY;	

		x = cvRound(d_x);
		y = cvRound(d_y);

		//�ڹ���ڵ��˳���
		if ((x >= 0) && (x < imgWidth) && (y >= 0) && (y < imgHeight))	
		{
				//ֻͳ���µ�����
				if (x != oldX || y != oldY)
				{
					_img.data[y * imgWidth + x] = 255;
					oldX = x;
					oldY = y;

				}
				
				
		}

	}//for theta
	
	return ;
}

/* ԭͼ������� */
void PupilDetection::SrcImgDetectPupil(void)
{
	m_srcImgWidth = m_srcImg.cols;
	m_srcImgHeight = m_srcImg.rows;
	double ratio = ((double)m_srcImgWidth / IMGWIDTH + (double)m_srcImgHeight / IMGHEIGHT) / 2;

	m_srcImgOffset = m_offset * ratio;
	
	int startA = m_ellipse.a * ratio - SRCIMG_A_EXTENT;
	int startB = max(m_ellipse.b * ratio - SRCIMG_B_EXTENT, startA * RATIO_THRESHOLD);
	int startX = m_ellipse.x * ratio - SRCIMG_X_EXTENT;
	int startY = m_ellipse.y * ratio - SRCIMG_Y_EXTENT;
	int startAlphaK = (m_ellipse.alpha - SRCIMG_ALPHA_EXTENT) / ALPHA_INCREMENT;

	int endA = m_ellipse.a * ratio + SRCIMG_A_EXTENT;
	int endB = m_ellipse.b * ratio + SRCIMG_B_EXTENT;
	int endX = m_ellipse.x * ratio + SRCIMG_X_EXTENT;
	int endY = m_ellipse.y * ratio + SRCIMG_Y_EXTENT;
	int endAlphaK = (m_ellipse.alpha + SRCIMG_ALPHA_EXTENT) / ALPHA_INCREMENT + 1;
/*
	//a �����߽�
	if (startA < A_B_THRESHOLD)startA = A_B_THRESHOLD;
	if (endA > (m_srcImgWidth - SRCIMG_FRAME_THRESHOLD) / 2)endA = (m_srcImgWidth - SRCIMG_FRAME_THRESHOLD) / 2;
	//x �����߽�
	if (startX < SRCIMG_FRAME_THRESHOLD)startX = SRCIMG_FRAME_THRESHOLD;
	if (endX > m_srcImgWidth - SRCIMG_FRAME_THRESHOLD)endX = m_srcImgWidth - SRCIMG_FRAME_THRESHOLD;
	//y �����߽�
	if (startY < SRCIMG_FRAME_THRESHOLD)startY = SRCIMG_FRAME_THRESHOLD;
	if (endY > m_srcImgWidth - SRCIMG_FRAME_THRESHOLD)endY = m_srcImgWidth - SRCIMG_FRAME_THRESHOLD;
*/
	//a �����߽�
	if (startA < A_B_THRESHOLD)startA = A_B_THRESHOLD;
	if (endA >(min(m_maxX - m_minX, m_maxY - m_minY) / 2))endA = min(m_maxX - m_minX, m_maxY - m_minY) / 2;
	//x �����߽�
	if (startX < m_minX)startX = m_minX;
	if (endX > m_maxX)endX = m_maxX;
	//y �����߽�
	if (startY < m_minY)startY = m_minY;
	if (endY > m_maxY)endY = m_maxY;

	int alphaKLen = PD_PI / ALPHA_INCREMENT + 1;
	int tempAlphaK = 0;
	int minX, minY, maxX, maxY = 0;
	double maxVal = 0;
	double tempVal = 0;
	int tempPointCount, maxPointCount = 0;
	vector<PDThreePoints> vecPoints;

	for (int a = startA; a < endA; a ++)
	{
		for (int b = startB; b < min(endB, a); b ++)
		{
			for (int alphaK = startAlphaK; alphaK < endAlphaK; alphaK ++)
			{
				if (alphaK < 0)
				{
					alphaK += alphaKLen;
				}
				//�������a, b, alpha�µ���ԲԲ�ܸ���
				tempPointCount = SrcImgPeripheryPointCalcu(a, b, alphaK, minX, maxX, minY, maxY);
				if (tempPointCount < 0)
				{
					continue;
				}
//				if (tempPointCount > maxPointCount)maxPointCount = tempPointCount;
				minX = max(m_minX - minX + OUT_OFFSET, startX);
				maxX = min(m_maxX - maxX - OUT_OFFSET, endX);
				minY = max(m_minY - minY + OUT_OFFSET, startY);
				maxY = min(m_maxY - maxY - OUT_OFFSET, endY);

				for (int x = minX; x < maxX; x ++)
				{
					for (int y = minY; y < maxY; y ++)
					{
						tempVal = SrcImgEllipseCalcu(x, y, tempPointCount);

						if (maxVal < tempVal)
						{
							maxVal = tempVal;
							m_ellipse.x = x;
							m_ellipse.y = y;
							m_ellipse.a = a;
							m_ellipse.b = b;
							tempAlphaK = alphaK;
						//	m_ellipse.alpha = ALPHA_INCREMENT * alphaK;
						}//if
					}//y
				}//x
			}//alpha
			if (a == b)continue;
		}//b
	}//a
	m_ellipse.alpha = ALPHA_INCREMENT * tempAlphaK;
//	cout << "maxVal: " << maxVal << endl;
//	cout << "maxPointCount: " << maxPointCount << endl;
	//�滭��Բ
	PDDrawEllipse(m_srcImg, m_srcImgWidth, m_srcImgHeight);

}

/* ������Բ�߽����ص��ݶ�ƽ��ֵ */
double PupilDetection::SrcImgEllipseCalcu(const int _x, const int _y, const int count)
{
//	int count = 0;
	double valueAccum = 0;

	//������ĻҶ�ֵ����һ��ֵ�����ų��������
	if ((m_srcImg.data[_y * m_srcImgWidth + _x]) > CENTER_THRESHOLD)
	{
		return -1;
	}
	
	int x = 0, y = 0;
	int increment = _y * m_srcImgWidth + _x;
	int imgOutElement = 0;
	int imgInElement = 0;
	int tempOutElement = 0;
	int tempInElement = 0;
	int outPos, inPos;
	
//	const int count = vecPoints.size();
	PDThreePoints tempP;
	for (int k = 0; k < count; k ++)
	{
		//��Բ�ϵ�����ļ���
		tempP = m_srcPeripheryPoints[k];
		outPos = tempP.outPointVal + increment;
		inPos = tempP.inPointVal + increment;

		tempOutElement = m_srcImg.data[outPos];
		tempInElement = m_srcImg.data[inPos];

		imgOutElement += tempOutElement;
		imgInElement += tempInElement;	
		
	}//for theta
	valueAccum = imgOutElement - imgInElement;

	return valueAccum / count;
}

/* �������a, b, alpha�µ���ԲԲ�ܸ��� */
int PupilDetection::SrcImgPeripheryPointCalcu(const int a, const int b, const int alphaK, 
										 int &minX, int &maxX, int &minY, int &maxY)
{
//	vecPoints.clear();

	int x = -1, y = -1;
	double d_x = -1, d_y = -1;
	int oldX = -1, oldY = -1;
	int out_x, out_y, in_x, in_y;
	PDThreePoints tempPoint;
	int count = 0;
	minX = 0;
	maxX = 0;
	minY = 0;
	maxY = 0;

	const double COS_OUT_B = m_cosAlpha[alphaK] * b;
	const double COS_OUT_A = m_cosAlpha[alphaK] * a;
	const double SIN_OUT_B = m_sinAlpha[alphaK] * b;
	const double SIN_OUT_A = m_sinAlpha[alphaK] * a;

	const int MAX_K = TWOPI / THETA_INCREMENT;

	for (int k = 0; k < MAX_K; k ++)
	{
		//����Բ�ϵ�����ļ���
		d_x = COS_OUT_B * m_cosTheta[k] - SIN_OUT_A * m_sinTheta[k];
		d_y = SIN_OUT_B * m_cosTheta[k] + COS_OUT_A * m_sinTheta[k];

		x = cvRound(d_x);
		y = cvRound(d_y);
/*
		x = int(d_x + 0.5);
		y = int(d_y + 0.5);
*/
		//ֻͳ���µ�����
		if (x != oldX || y != oldY)
		{
			//����  Ϊ��
			if (x >= 0)
			{
				out_x = x + OUT_OFFSET;
				in_x = x - IN_OFFSET;

				if (out_x > maxX)maxX = out_x;
			}
			else
			{
				out_x = x - OUT_OFFSET;
				in_x = x + IN_OFFSET;

				if (in_x < minX)minX = in_x;
			}
			//����  Ϊ��
			if (y >= 0)
			{
				out_y = y + OUT_OFFSET;
				in_y = y - IN_OFFSET;

				if (out_y > maxY)maxY = out_y;
			}
			else
			{
				out_y = y - OUT_OFFSET;
				in_y = y + IN_OFFSET;

				if (out_y < minY)minY = out_y;
			}
			tempPoint.outPointVal = out_y * m_srcImgWidth + out_x;
			tempPoint.inPointVal = in_y * m_srcImgWidth + in_x;
			m_srcPeripheryPoints[count] = tempPoint;
			count ++;
//			vecPoints.push_back(tempPoint);
			oldX = x;
			oldY = y;
		}
	
	}

//	const int count = vecPoints.size();
	if (count < SRCIMG_MIN_POINT)return -1;

	return count;
}

/* �Դ���ͼ����д��� */
int PupilDetection::ImgProcessing(void)
{
	if (NULL == m_img.data)
	{
#if _DEBUG
		cout << "ͼ��Ϊ�գ�" << endl;
#endif
		return FAILED;
	}


	if (1 != m_img.channels())
	{
		//�Ҷ�ͼ
		cvtColor(m_img, m_img, CV_BGR2GRAY);
	}
	//��ֵ��
	Mat thresholdImg;
	threshold(m_img, thresholdImg, G_L_THRESHOLD, 255, THRESH_BINARY);
	//������
	vector<vector<Point>> contours;
	findContours(thresholdImg, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);

	const int offset1 = 1;
	const int offset2 = 3;
	int width = m_img.cols;
	int height = m_img.rows;
	int topElement, bottomElement, leftElement, rightElement;
	int avgElement;
	int elements[4];
	int highEle[4] = {-1}, lowEle[4] = {-1};
	int maxOffset1 = 0, maxOffset2 = 0;
	int tempOffset1 = 0, tempOffset2 = 0;
	int x, y, r, R;
	int A, B, C;
	int rOffset;
	CvPoint linePoint1, linePoint2;
	CvPoint leftTopPoint, rightBottomPoint;
	Point2f circleCenter;
	float cirCleRadius;
	int contoursSize = contours.size();
	int i = 0, h = 0, l = 0, t = 0, j = 0, k = 0;
	for (i = 0; i < contoursSize; i ++)
	{
		if (contours[i].size() < CONTOURS_MIN_SIZE || contours[i].size() > CONTOURS_MAX_SIZE)
		{
			continue;
		}

		minEnclosingCircle(contours[i], circleCenter, cirCleRadius);
		x = circleCenter.x;
		y = circleCenter.y;
		r = cvRound(cirCleRadius) + G_L_OFFSET;

		if ((x + 2 * r > width || x - 2 * r < 0) || (y + 2 * r > height || y - 2 * r < 0))
		{
			continue;
		}

		topElement = m_img.data[(y - r) * width + x];
		bottomElement = m_img.data[(y + r) * width + x];
		leftElement = m_img.data[y * width + (x - r)];
		rightElement = m_img.data[y * width + (x + r)];

		elements[0] = topElement;
		elements[1] = rightElement;
		elements[2] = bottomElement;
		elements[3] = leftElement;

		leftTopPoint.x = x - r;
		leftTopPoint.y = y - r;
		rightBottomPoint.x = x + r;
		rightBottomPoint.y = y + r;

		int maxElement = 0;
		//�����ֵ
		for (t = 1; t < 4; t ++)
		{
			if (elements[maxElement] < elements[t])maxElement = t;
		}
		//����
		h = 0, l = 0;
		for (t = 0; t < 4; t ++)
		{
			highEle[t] = -1;
			lowEle[t] = -1;
			if (abs(elements[maxElement] - elements[t]) < G_L_DIFF_TH)
			{
				highEle[h] = t;
				h ++;
			}
			else
			{
				lowEle[l] = t;
				l ++;
			}
		}
		if (3 == lowEle[1])
		{
			int tempEle = lowEle[0];
			lowEle[0] = lowEle[1];
			lowEle[1] = tempEle;
		}
		rOffset = 2 * r;
		switch (h)
		{
		case 4:		//(4, 0)(�ߣ� ��)
			avgElement = (topElement + leftElement + bottomElement + rightElement) / 4;
			for (j = x - r; j < x + r; j ++)
			{
				for (k = y - r; k < y + r; k ++)
				{
					m_img.data[(k)* width + j] = avgElement;
				}
			}
			break;

		case 3:		//(3, 1)
			
			switch (lowEle[0])
			{
				//���¶���δ���������ճ��ʱ����ǲ�����Ҫ�Ķ������Ϊ /* @_n */
			case 0:		//top
				/* @_0 leftElement,bottomElement,rightElement��ѡ�� */
				avgElement = (leftElement + bottomElement + rightElement) / 3;
				maxOffset1 = 0;
				maxOffset2 = 0;
				tempOffset1 = 0;
				tempOffset2 = 0;
				/* @_2 ��k�ı����Լ�(k + offset) * width + linePoint1.x�ȵļ��㣬���ж�linePoint1, linePoint2��x, yֵ */
				linePoint1.x = x - r;
				linePoint2.x = x + r;
				for (k = y - rOffset; k < y; k ++)
				{
					tempOffset1 = m_img.data[(k + offset1) * width + linePoint1.x] - m_img.data[(k - offset2) * width + linePoint1.x];
					tempOffset2 = m_img.data[(k + offset1) * width + linePoint2.x] - m_img.data[(k - offset2) * width + linePoint2.x];
					if (tempOffset1 > maxOffset1)
					{
						maxOffset1 = tempOffset1;
						linePoint1.y = k;
					}
					if (tempOffset2 > maxOffset2)
					{
						maxOffset2 = tempOffset2;
						linePoint2.y = k;
					}
				}
				//��������ȷ����ֱ��
				LinearEquationCalcu(linePoint1, linePoint2, A, B, C);
				if (A * B >= 0)
				{
					SpotAreaFill(m_img, leftTopPoint, rightBottomPoint, elements[0], avgElement, A, B, C);
				}
				else
				{
					SpotAreaFill(m_img, leftTopPoint, rightBottomPoint, avgElement, elements[0], A, B, C);
				}
				break;

			case 1:		//right
				/* @_0 leftElement,bottomElement,rightElement��ѡ�� */
				avgElement = (leftElement + bottomElement + topElement) / 3;
				maxOffset1 = 0;
				maxOffset2 = 0;
				tempOffset1 = 0;
				tempOffset2 = 0;
				/* @_2 ��k�ı����Լ�(k + offset) * width + linePoint1.x�ȵļ��㣬���ж�linePoint1, linePoint2��x, yֵ */
				linePoint1.y = y - r;
				linePoint2.y = y + r;
				for (j = x; j < x + rOffset; j ++)
				{
					tempOffset1 = m_img.data[linePoint1.y * width + j - offset1] - m_img.data[linePoint1.y * width + j + offset2];
					tempOffset2 = m_img.data[linePoint2.y * width + j - offset1] - m_img.data[linePoint2.y * width + j + offset2];
					if (tempOffset1 > maxOffset1)
					{
						maxOffset1 = tempOffset1;
						linePoint1.x = j;
					}
					if (tempOffset2 > maxOffset2)
					{
						maxOffset2 = tempOffset2;
						linePoint2.x = j;
					}
				}
				//��������ȷ����ֱ��
				LinearEquationCalcu(linePoint1, linePoint2, A, B, C);
				SpotAreaFill(m_img, leftTopPoint, rightBottomPoint, avgElement, elements[1], A, B, C);
				break;

			case 2:		//bottom
				/* @_0 leftElement,bottomElement,rightElement��ѡ�� */
				avgElement = (leftElement + topElement + rightElement) / 3;
				maxOffset1 = 0;
				maxOffset2 = 0;
				tempOffset1 = 0;
				tempOffset2 = 0;
				/* @_2 ��k�ı����Լ�(k + offset) * width + linePoint1.x�ȵļ��㣬���ж�linePoint1, linePoint2��x, yֵ */
				linePoint1.x = x - r;
				linePoint2.x = x + r;
				for (k = y; k < y + rOffset; k ++)
				{
					tempOffset1 = m_img.data[(k - offset1) * width + linePoint1.x] - m_img.data[(k + offset2) * width + linePoint1.x];
					tempOffset2 = m_img.data[(k - offset1) * width + linePoint2.x] - m_img.data[(k + offset2) * width + linePoint2.x];
					if (tempOffset1 > maxOffset1)
					{
						maxOffset1 = tempOffset1;
						linePoint1.y = k;
					}
					if (tempOffset2 > maxOffset2)
					{
						maxOffset2 = tempOffset2;
						linePoint2.y = k;
					}
				}
				//��������ȷ����ֱ��
				LinearEquationCalcu(linePoint1, linePoint2, A, B, C);
				if (A * B >= 0)
				{
					SpotAreaFill(m_img, leftTopPoint, rightBottomPoint, avgElement, elements[0], A, B, C);
				}
				else
				{
					SpotAreaFill(m_img, leftTopPoint, rightBottomPoint, elements[2], avgElement, A, B, C);
				}
				break;

			case 3:		//right
				/* @_0 leftElement,bottomElement,rightElement��ѡ�� */
				avgElement = (rightElement + bottomElement + topElement) / 3;
				maxOffset1 = 0;
				maxOffset2 = 0;
				tempOffset1 = 0;
				tempOffset2 = 0;
				/* @_2 ��k�ı����Լ�(k + offset) * width + linePoint1.x�ȵļ��㣬���ж�linePoint1, linePoint2��x, yֵ */
				linePoint1.y = y - r;
				linePoint2.y = y + r;
				for (j = x; j < x + rOffset; j ++)
				{
					tempOffset1 = m_img.data[linePoint1.y * width + j + offset1] - m_img.data[linePoint1.y * width + j - offset2];
					tempOffset2 = m_img.data[linePoint2.y * width + j + offset1] - m_img.data[linePoint2.y * width + j - offset2];
					if (tempOffset1 > maxOffset1)
					{
						maxOffset1 = tempOffset1;
						linePoint1.x = j;
					}
					if (tempOffset2 > maxOffset2)
					{
						maxOffset2 = tempOffset2;
						linePoint2.x = j;
					}
				}
				//��������ȷ����ֱ��
				LinearEquationCalcu(linePoint1, linePoint2, A, B, C);
				SpotAreaFill(m_img, leftTopPoint, rightBottomPoint, elements[3], avgElement, A, B, C);
				break;

			default:
				break;
			}
			break;

		case 2:		//(2, 2)
			int filledVal1, filledVal2;
			filledVal1 = (elements[highEle[0]] + elements[highEle[1]]) / 2;
			filledVal2 = (elements[lowEle[0]] + elements[lowEle[1]]) / 2;
			int rOffset1, rOffset2;
			rOffset1 = 2 * r;
			rOffset2 = r;
			CvPoint tempP1, tempP2;
			switch (lowEle[0])
			{
				//���¶���δ���������ճ��ʱ����ǲ�����Ҫ�Ķ������Ϊ /* @_n */
			case 0:		//top			
				maxOffset1 = 0;
				maxOffset2 = 0;
				tempOffset1 = 0;
				tempOffset2 = 0;
				tempP1.x = x - rOffset1;
				tempP1.y = y - rOffset2;
				tempP2.x = x + rOffset2;
				tempP2.y = y + rOffset1;
				for (k = 0; k < rOffset1; k ++)
				{
					tempOffset1 = m_img.data[(tempP1.y + offset1) * width + tempP1.x - offset1]
						- m_img.data[(tempP1.y - offset2) * width + tempP1.x + offset2];
					tempOffset2 = m_img.data[(tempP2.y + offset1) * width + tempP2.x - offset1]
						- m_img.data[(tempP2.y - offset2) * width + tempP2.x + offset2];
					if (tempOffset1 > maxOffset1)
					{
						maxOffset1 = tempOffset1;
						linePoint1.x = tempP1.x;
						linePoint1.y = tempP1.y;
					}
					if (tempOffset2 > maxOffset2)
					{
						maxOffset2 = tempOffset2;
						linePoint2.x = tempP2.x;
						linePoint2.y = tempP2.y;
					}
					tempP1.x ++;
					tempP1.y --;
					tempP2.x ++;
					tempP2.y --;
				}
				//��������ȷ����ֱ��
				LinearEquationCalcu(linePoint1, linePoint2, A, B, C);
				SpotAreaFill(m_img, leftTopPoint, rightBottomPoint, filledVal1, filledVal2, A, B, C);
				break;

			case 1:		//right
				maxOffset1 = 0;
				maxOffset2 = 0;
				tempOffset1 = 0;
				tempOffset2 = 0;
				tempP1.x = x - rOffset1;
				tempP1.y = y + rOffset2;
				tempP2.x = x + rOffset2;
				tempP2.y = y - rOffset1;
				for (k = 0; k < rOffset1; k ++)
				{
					tempOffset1 = m_img.data[(tempP1.y - offset1) * width + tempP1.x - offset1]
						- m_img.data[(tempP1.y + offset2) * width + tempP1.x + offset2];
					tempOffset2 = m_img.data[(tempP2.y - offset1) * width + tempP2.x - offset1]
						- m_img.data[(tempP2.y + offset2) * width + tempP2.x + offset2];
					if (tempOffset1 > maxOffset1)
					{
						maxOffset1 = tempOffset1;
						linePoint1.x = tempP1.x;
						linePoint1.y = tempP1.y;
					}
					if (tempOffset2 > maxOffset2)
					{
						maxOffset2 = tempOffset2;
						linePoint2.x = tempP2.x;
						linePoint2.y = tempP2.y;
					}
					tempP1.x ++;
					tempP1.y ++;
					tempP2.x ++;
					tempP2.y ++;
				}
				//��������ȷ����ֱ��
				LinearEquationCalcu(linePoint1, linePoint2, A, B, C);
				SpotAreaFill(m_img, leftTopPoint, rightBottomPoint, filledVal1, filledVal2, A, B, C);
				break;

			case 2:		//bottom
				maxOffset1 = 0;
				maxOffset2 = 0;
				tempOffset1 = 0;
				tempOffset2 = 0;
				tempP1.x = x + rOffset1;
				tempP1.y = y + rOffset2;
				tempP2.x = x + rOffset2;
				tempP2.y = y - rOffset1;
				for (k = 0; k < rOffset1; k ++)
				{
					tempOffset1 = m_img.data[(tempP1.y - offset1) * width + tempP1.x + offset1]
						- m_img.data[(tempP1.y + offset2) * width + tempP1.x - offset2];
					tempOffset2 = m_img.data[(tempP2.y - offset1) * width + tempP2.x + offset1]
						- m_img.data[(tempP2.y + offset2) * width + tempP2.x - offset2];
					if (tempOffset1 > maxOffset1)
					{
						maxOffset1 = tempOffset1;
						linePoint1.x = tempP1.x;
						linePoint1.y = tempP1.y;
					}
					if (tempOffset2 > maxOffset2)
					{
						maxOffset2 = tempOffset2;
						linePoint2.x = tempP2.x;
						linePoint2.y = tempP2.y;
					}
					tempP1.x --;
					tempP1.y ++;
					tempP2.x --;
					tempP2.y ++;
				}
				//��������ȷ����ֱ��
				LinearEquationCalcu(linePoint1, linePoint2, A, B, C);
				SpotAreaFill(m_img, leftTopPoint, rightBottomPoint, filledVal2, filledVal1, A, B, C);
				break;

			case 3:		//right
				maxOffset1 = 0;
				maxOffset2 = 0;
				tempOffset1 = 0;
				tempOffset2 = 0;
				tempP1.x = x + rOffset1;
				tempP1.y = y - rOffset2;
				tempP2.x = x - rOffset2;
				tempP2.y = y + rOffset1;
				for (k = 0; k < rOffset1; k ++)
				{
					tempOffset1 = m_img.data[(tempP1.y + offset1) * width + tempP1.x + offset1]
						- m_img.data[(tempP1.y - offset2) * width + tempP1.x - offset2];
					tempOffset2 = m_img.data[(tempP2.y + offset1) * width + tempP2.x + offset1]
						- m_img.data[(tempP2.y - offset2) * width + tempP2.x - offset2];
					if (tempOffset1 > maxOffset1)
					{
						maxOffset1 = tempOffset1;
						linePoint1.x = tempP1.x;
						linePoint1.y = tempP1.y;
					}
					if (tempOffset2 > maxOffset2)
					{
						maxOffset2 = tempOffset2;
						linePoint2.x = tempP2.x;
						linePoint2.y = tempP2.y;
					}
					tempP1.x --;
					tempP1.y --;
					tempP2.x --;
					tempP2.y --;
				}
				//��������ȷ����ֱ��
				LinearEquationCalcu(linePoint1, linePoint2, A, B, C);
				SpotAreaFill(m_img, leftTopPoint, rightBottomPoint, filledVal2, filledVal1, A, B, C);
				break;

			default:
				break;
			}
			break;

		case 1:		//(1, 3)
			switch (highEle[0])
			{
				//���¶���δ���������ճ��ʱ����ǲ�����Ҫ�Ķ������Ϊ /* @_n */
			case 0:		//top
				/* @_0 leftElement,bottomElement,rightElement��ѡ�� */
				avgElement = (leftElement + bottomElement + rightElement) / 3;
				maxOffset1 = 0;
				maxOffset2 = 0;
				tempOffset1 = 0;
				tempOffset2 = 0;
				/* @_2 ��k�ı����Լ�(k + offset) * width + linePoint1.x�ȵļ��㣬���ж�linePoint1, linePoint2��x, yֵ */
				linePoint1.x = x - r;
				linePoint2.x = x + r;
				for (k = y - rOffset; k < y; k ++)
				{
					tempOffset1 = m_img.data[(k - offset1) * width + linePoint1.x] - m_img.data[(k + offset2) * width + linePoint1.x];
					tempOffset2 = m_img.data[(k - offset1) * width + linePoint2.x] - m_img.data[(k + offset2) * width + linePoint2.x];
					if (tempOffset1 > maxOffset1)
					{
						maxOffset1 = tempOffset1;
						linePoint1.y = k;
					}
					if (tempOffset2 > maxOffset2)
					{
						maxOffset2 = tempOffset2;
						linePoint2.y = k;
					}
				}
				//��������ȷ����ֱ��
				LinearEquationCalcu(linePoint1, linePoint2, A, B, C);
				if (A * B >= 0)
				{
					SpotAreaFill(m_img, leftTopPoint, rightBottomPoint, elements[0], avgElement, A, B, C);
				}
				else
				{
					SpotAreaFill(m_img, leftTopPoint, rightBottomPoint, avgElement, elements[0], A, B, C);
				}
				break;

			case 1:		//right
				/* @_0 leftElement,bottomElement,rightElement��ѡ�� */
				avgElement = (leftElement + bottomElement + topElement) / 3;
				maxOffset1 = 0;
				maxOffset2 = 0;
				tempOffset1 = 0;
				tempOffset2 = 0;
				/* @_2 ��k�ı����Լ�(k + offset) * width + linePoint1.x�ȵļ��㣬���ж�linePoint1, linePoint2��x, yֵ */
				linePoint1.y = y - r;
				linePoint2.y = y + r;
				for (j = x; j < x + rOffset; j ++)
				{
					tempOffset1 = m_img.data[linePoint1.y * width + j + offset1] - m_img.data[linePoint1.y * width + j - offset2];
					tempOffset2 = m_img.data[linePoint2.y * width + j + offset1] - m_img.data[linePoint2.y * width + j - offset2];
					if (tempOffset1 > maxOffset1)
					{
						maxOffset1 = tempOffset1;
						linePoint1.x = j;
					}
					if (tempOffset2 > maxOffset2)
					{
						maxOffset2 = tempOffset2;
						linePoint2.x = j;
					}
				}
				//��������ȷ����ֱ��
				LinearEquationCalcu(linePoint1, linePoint2, A, B, C);
				SpotAreaFill(m_img, leftTopPoint, rightBottomPoint, avgElement, elements[1], A, B, C);
				break;

			case 2:		//bottom
				/* @_0 leftElement,bottomElement,rightElement��ѡ�� */
				avgElement = (leftElement + topElement + rightElement) / 3;
				maxOffset1 = 0;
				maxOffset2 = 0;
				tempOffset1 = 0;
				tempOffset2 = 0;
				/* @_2 ��k�ı����Լ�(k + offset) * width + linePoint1.x�ȵļ��㣬���ж�linePoint1, linePoint2��x, yֵ */
				linePoint1.x = x - r;
				linePoint2.x = x + r;
				for (k = y; k < y + rOffset; k ++)
				{
					tempOffset1 = m_img.data[(k + offset1) * width + linePoint1.x] - m_img.data[(k - offset2) * width + linePoint1.x];
					tempOffset2 = m_img.data[(k + offset1) * width + linePoint2.x] - m_img.data[(k - offset2) * width + linePoint2.x];
					if (tempOffset1 > maxOffset1)
					{
						maxOffset1 = tempOffset1;
						linePoint1.y = k;
					}
					if (tempOffset2 > maxOffset2)
					{
						maxOffset2 = tempOffset2;
						linePoint2.y = k;
					}
				}
				//��������ȷ����ֱ��
				LinearEquationCalcu(linePoint1, linePoint2, A, B, C);
				if (A * B >= 0)
				{
					SpotAreaFill(m_img, leftTopPoint, rightBottomPoint, avgElement, elements[0], A, B, C);
				}
				else
				{
					SpotAreaFill(m_img, leftTopPoint, rightBottomPoint, elements[2], avgElement, A, B, C);
				}
				break;

			case 3:		//right
				/* @_0 leftElement,bottomElement,rightElement��ѡ�� */
				avgElement = (rightElement + bottomElement + topElement) / 3;
				maxOffset1 = 0;
				maxOffset2 = 0;
				tempOffset1 = 0;
				tempOffset2 = 0;
				/* @_2 ��k�ı����Լ�(k + offset) * width + linePoint1.x�ȵļ��㣬���ж�linePoint1, linePoint2��x, yֵ */
				linePoint1.y = y - r;
				linePoint2.y = y + r;
				for (j = x; j < x + rOffset; j ++)
				{
					tempOffset1 = m_img.data[linePoint1.y * width + j - offset1] - m_img.data[linePoint1.y * width + j + offset2];
					tempOffset2 = m_img.data[linePoint2.y * width + j - offset1] - m_img.data[linePoint2.y * width + j + offset2];
					if (tempOffset1 > maxOffset1)
					{
						maxOffset1 = tempOffset1;
						linePoint1.x = j;
					}
					if (tempOffset2 > maxOffset2)
					{
						maxOffset2 = tempOffset2;
						linePoint2.x = j;
					}
				}
				//��������ȷ����ֱ��
				LinearEquationCalcu(linePoint1, linePoint2, A, B, C);
				SpotAreaFill(m_img, leftTopPoint, rightBottomPoint, elements[3], avgElement, A, B, C);
				break;

			default:
				break;
			}
			break;

		default:
			break;
		}
		/*
		cout << "�� " << i << "�� " << endl;
		imshow("1", m_img);
		waitKey(1000);
		*/

	}

	//ͫ�״ֶ�λ
	PupilCoarsePositioning();

	//����ԭͼ
	m_img.copyTo(m_srcImg);
	//�ߴ��һ��
	resize(m_img, m_img, Size(IMGWIDTH, IMGHEIGHT));

	return SUCCEED;
}

/* �����������ȷ����ֱ�� A * x + B * y + C = 0 */
void PupilDetection::LinearEquationCalcu(CvPoint &point1, CvPoint &point2, int &A, int &B, int &C)
{
	A = point2.y - point1.y;
	B = point1.x - point2.x;
	C = point2.x * point1.y - point1.x * point2.y;
	if (A > 0)
	{
		return;
	}
	else if (A < 0)
	{
		A = - A;
		B = - B;
		C = - C;
		return;
	}
	else if (B < 0)
	{
		B = - B;
		C = - C;
		return;
	}
}

/* �Թ������������,�� A*x + B*y + C < 0�������filledVal1 */
void PupilDetection::SpotAreaFill(Mat &img, CvPoint leftTopPoint, CvPoint rightBottomPoint, int filledVal1, int filledVal2,
	int A, int B, int C)
{
	int width = img.cols;
	int a = 0, b = 0, c = 0;

	for (int x = leftTopPoint.x; x < rightBottomPoint.x; x ++)
	{
		for (int y = leftTopPoint.y; y < rightBottomPoint.y; y ++)
		{
			if (A * x + B * y + C < 0)
			{
				img.data[y * width + x] = filledVal1;
				a ++;
			}
			else if (A * x + B * y + C == 0)
			{
				img.data[y * width + x] = (filledVal1 + filledVal2) / 2;
				b ++;
			}
			else
			{
				img.data[y * width + x] = filledVal2;
				c ++;
			}
		}
	}
//	cout << "a = " << a << "; b = " << b << "; c = " << c << endl;
}

/* ͫ�״ֶ�λ */
void PupilDetection::PupilCoarsePositioning()
{
	//��ֵ��
	Mat thresholdImg;
	threshold(m_img, thresholdImg, 100, 255, CV_THRESH_BINARY_INV);
	
	//��ͼ��ƽ������
	GaussianBlur(thresholdImg, thresholdImg, Size(5, 5), 0);
//	Mat tempImg;
//	thresholdImg.copyTo(tempImg);
	/*
	imshow("0", thresholdImg);
	waitKey(2000);
	*/
	//������
	vector<vector<Point>> contours;
	findContours(thresholdImg, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);


	int width = m_img.cols;
	int height = m_img.rows;
	const double ratioR = 1.5;
	Point circleCenter(0, 0), tempCircleCenter;
	int circleRadius = 0, tempCircleRadius;
	int contoursSize = contours.size();
	int i = 0, h = 0, l = 0, t = 0, j = 0, k = 0;
	int minX = SRCIMG_FRAME_THRESHOLD;
	int maxX = m_img.cols - SRCIMG_FRAME_THRESHOLD;
	int minY = SRCIMG_FRAME_THRESHOLD;
	int maxY = m_img.rows - SRCIMG_FRAME_THRESHOLD;
	double valueAccum = -1, tempValueAccum;
	bool changedLabel = false;

	for (i = 0; i < contoursSize; i ++)
	{
		if (ContourScreening(contours[i], tempValueAccum, tempCircleCenter, tempCircleRadius))
		{
			//��Բ
//			PDDrawCircle(tempImg, tempCircleRadius, tempCircleCenter.x, tempCircleCenter.y);
			//��һ��
			if (!changedLabel)
			{
				valueAccum = tempValueAccum;
				circleCenter.x = tempCircleCenter.x;
				circleCenter.y = tempCircleCenter.y;
				circleRadius = tempCircleRadius;
				changedLabel = true;
				continue;
			}
			if (valueAccum > tempValueAccum)
			{
				valueAccum = tempValueAccum;
				circleCenter.x = tempCircleCenter.x;
				circleCenter.y = tempCircleCenter.y;
				circleRadius = tempCircleRadius;
			}
			
		}
		/*
		PDDrawRect(m_img, Point(circleCenter.x - circleRadius, circleCenter.y - circleRadius), 
			Point(circleCenter.x + circleRadius, circleCenter.y + circleRadius));
			*/
			
	}
	/*
	imshow("1", tempImg);
	waitKey(2000);
	*/
	//�ֶ�λ��Ч
	if (changedLabel)
	{
		m_minX = max(int(circleCenter.x - ratioR * circleRadius), minX);
		m_maxX = min(int(circleCenter.x + ratioR * circleRadius), maxX);
		m_minY = max(int(circleCenter.y - ratioR * circleRadius), minY);
		m_maxY = min(int(circleCenter.y + ratioR * circleRadius), maxY);
	}
//	cout << m_minX << " , " << m_maxX << " ; " << m_minY << " , " << m_maxY << endl;


//	PDDrawRect(m_img, Point(minX, minY), Point(maxX, maxY));
	/*
	imshow("1", m_tempImg);
	waitKey(0);
	*/
	return;
}

/* ����ͫ�״ֶ�λʱ������ɸѡ */
bool PupilDetection::ContourScreening(vector<Point> &contours, double &valueAccum, Point &circleCenter, int &circleRadius)
{

	Point2f circleC;
	float circleR;
	int contoursSize = contours.size();
	int x, y, r, R;
	int i, j;
	double ratio;
	int tempMinX, tempMinY, tempMaxX, tempMaxY;

	if (!contoursSize)
	{
		return false;
	}

	//��������Ӧ��һ����Χ��
	if (contoursSize < 200 || contoursSize > 500)
	{
		return false;
	}

	minEnclosingCircle(contours, circleC, circleR);
	x = circleC.x;
	y = circleC.y;
	r = cvRound(circleR);

	tempMinX = contours[0].x;
	tempMaxX = contours[0].x;
	tempMinY = contours[0].y;
	tempMaxY = contours[0].y;
	for (j = 1; j < contoursSize; j ++)
	{
		if (tempMinX > contours[j].x)tempMinX = contours[j].x;
		if (tempMaxX < contours[j].x)tempMaxX = contours[j].x;
		if (tempMinY > contours[j].y)tempMinY = contours[j].y;
		if (tempMaxY < contours[j].y)tempMaxY = contours[j].y;
	}
	ratio = double(tempMaxY - tempMinY) / (tempMaxX - tempMinX);
	ratio = ratio < 1 ? ratio : 1 / ratio;
	//���ڱ�ƽ����ȥ
	if (ratio < RATIO_THRESHOLD)return false;

	//���������r�Ĳ�ֵ�ľ���ֵƽ��ֵ
	double diffX, diffY;
	valueAccum = 0;
	for (i = 0; i < contoursSize; i ++)
	{
		diffX = double(contours[i].x - x) / r;
		diffY = double(contours[i].y - y) / r;
//		valueAccum += abs(diffX * diffX + diffY * diffY - r * r);
		valueAccum += abs(diffX * diffX + diffY * diffY - 1);
	}
	valueAccum = valueAccum / contoursSize;

	circleCenter.x = x;
	circleCenter.y = y;
	circleRadius = r;

	return true;
}

void PDDrawRect(Mat &img, Point pt1, Point pt2)
{
	int i = 0, j = 0;
	int width = img.cols, height = img.rows;

	if (pt1.x < 0 || pt1.x > width || pt1.y < 0 || pt1.y > height)
	{
		return;
	}
	if (pt2.x < 0 || pt2.x > width || pt2.y < 0 || pt2.y > height)
	{
		return;
	}

	for (i = pt1.x; i < pt2.x; i ++)
	{
		img.data[pt1.y * width + i] = 255;
		img.data[pt2.y * width + i] = 255;
	}
	for (j = pt1.y; j < pt2.y; j ++)
	{
		img.data[j * width + pt1.x] = 255;
		img.data[j * width + pt2.x] = 255;
	}
	return;
}

void PupilDetection::PDDrawCircle(Mat &img, int r, int _x, int _y)
{
	int oldX = 0;
	int oldY = 0;
	int imgWidth = img.cols;

	int centerX = _x;
	int centerY = _y;

	int x = -1, y = -1;
	double d_x = -1, d_y = -1;
	const int MAX_K = TWOPI / THETA_INCREMENT;


	for (int k = 0; k < MAX_K; k ++)
	{
		//��Բ�ϵ�����ļ���
		d_x = r * m_cosTheta[k] + centerX;
		d_y = r * m_sinTheta[k] + centerY;

		x = cvRound(d_x);
		y = cvRound(d_y);

		
		//ֻͳ���µ�����
		if (x != oldX || y != oldY)
		{
			img.data[y * imgWidth + x] = 255;
			oldX = x;
			oldY = y;

		}

	}//for theta

	return;
}