
/****************************************************************
*	��Ŀ��ͫ�׼�ⶨλ											*
*	���ߣ��޼��� ���Ӻ�											*
*		  ������ ������ 										*
*	���ʱ�䣺													*
*																*
****************************************************************/

#ifndef PUPILDETECTION_H
#define PUPILDETECTION_H

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/contrib/contrib.hpp"

#include <time.h>
#include <math.h>
#include <iostream>
#include <sstream>
#include <fstream>

#define SUCCEED 1			//�ɹ�
#define FAILED -1			//ʧ��
#define PD_PI 3.1415926		
#define TWOPI 6.2831852

#define IMGHEIGHT 81		//ͼƬ�ߴ��һ���߶�
#define IMGWIDTH 108		//ͼƬ�ߴ��һ�����
#define SRCIMGHEIGHT 480	//ԭͼ�߶�
#define SRCIMGWIDTH 640		//ԭͼ���
#define GLINT_THRESHOLD 200			//�ų���ߵĻҶ�ֵ����ֵ
#define FRAME_THRESHOLD 10			//��ȡͼƬ�߿�Ŀ��
#define A_B_THRESHOLD 5				//��Բ����b����С����
#define RATIO_THRESHOLD 0.4			//��Բ�����볤��֮�ȵ���Сֵ
#define THETA_INCREMENT 0.05		//������ԲԲ�ܻҶ�ֵ��ƽ��ֵʱthetaֵ������
#define ALPHA_INCREMENT 0.3			//������Բalpha�Ƕ�ֵ������
#define CENTER_THRESHOLD 100		//ͫ�����ĻҶ�ֵ��ֵ�����ڸ�ֵ���ų�

using namespace std;  
using namespace cv; 

typedef struct		//��Բ
{
	int x;			//���ĺ�����
	int y;			//����������
	int a;			//����
	int b;			//����
	double alpha;		//ƫת�Ƕ�
}PDEllipse;

typedef struct		//4��������ݶ�ֵ
{
	int x;			//x
	int y;			//y
//	int gradDir;	//�����ݶȵķ����־
}PDPoint;

typedef struct		//�ݶ������������
{
	int outPointVal;
	int inPointVal;

}PDThreePoints;

class PupilDetection
{
public:
	//�ӿ�
	PupilDetection(Mat &_img, int _offset = 3);
	PupilDetection();
	~PupilDetection();
	/* ����offset */
	void setOffset(int _offset){m_offset = _offset; return;}
	/* ����ͼ�� */
	int setImg(Mat &_img);
	/* ����ͫ�׼�ⶨλ����⵽����Բ����ellipse�� */
	int DetectPupil(void);
	/* ����ellipse */
	void getEllipse(PDEllipse &_ellipse);
	/* ����ͼ�� */
	int getImg(Mat &_img);

private:
	/* �Դ���ͼ����д��� */
	int ImgProcessing(void);
	/* ����ͼ���ϸ���4��������ݶ�ֵ */
	void GradCalcu(void);
	/* �������a, b, alpha�µ���ԲԲ�ܸ��� */
	int PeripheryPointCalcu(const int a, const int b, const int alphaK, 
										 int &minX, int &maxX, int &minY, int &maxY);
	/* ���������Բ��ƫ�����µĲ�ּ��� */
	double EllipDiffCalcu(const int _x, const int _y, const int count);
	/* ������Բ�߽����ص�ƽ��ֵ */
	double EllipseCalcu(const int _x, const int _y, const int count);
	/* ������Բ */
	void PDDrawEllipse(Mat &_img, int imgWidth, int imgHeight);
	/* ԭͼ������� */
	void SrcImgDetectPupil(void);
	double SrcImgEllipseCalcu(const int _x, const int _y, const int count);
	/* �������a, b, alpha�µ���ԲԲ�ܸ��� */
	int SrcImgPeripheryPointCalcu(const int a, const int b, const int alphaK,
										 int &minX, int &maxX, int &minY, int &maxY);
	/* �����������ȷ����ֱ�� A * x + B * y + C = 0 */
	void LinearEquationCalcu(CvPoint &point1, CvPoint &point2, int &A, int &B, int &C);
	/* �Թ������������,�� A*x + B*y + C < 0�������filledVal1 */
	void SpotAreaFill(Mat &img, CvPoint leftTopPoint, CvPoint rightBottomPoint, int filledVal1, int filledVal2,
		int A, int B, int C);
	/* ͫ�״ֶ�λ */
	void PupilCoarsePositioning();
	/* ����ͫ�״ֶ�λʱ������ɸѡ */
	bool ContourScreening(vector<Point> &contours, double &valueAccum, Point &circleCenter, int &circleRadius);
	void PDDrawCircle(Mat &img, int r, int _x, int _y);

	//���ݱ���
	int m_offset;				//��Բ��ּ���ƫ����
	int m_srcImgOffset;			//ԭͼ��Բ��ּ���ƫ����
	PDEllipse m_ellipse;		//��Բ
	Mat m_img;					//ͼ��
	Mat m_srcImg;
	int m_minX;
	int m_maxX;
	int m_minY;
	int m_maxY;

	//�м����
	int m_peripheryPoints[2000];
	PDThreePoints m_srcPeripheryPoints[2000];
	int m_srcImgWidth;
	int m_srcImgHeight;
	int m_grads[IMGHEIGHT * IMGWIDTH];
	double *m_cosTheta;	//���cos theta��ֵ������
	double *m_sinTheta;	//���sin theta��ֵ������
	double *m_cosAlpha;	//���cos alpha��ֵ������
	double *m_sinAlpha;	//���sin alpha��ֵ������
};


#endif