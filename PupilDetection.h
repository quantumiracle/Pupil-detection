
/****************************************************************
*	项目：瞳孔检测定位											*
*	作者：罗家意 丁子涵											*
*		  田雨沛 章泽宇 										*
*	完成时间：													*
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

#define SUCCEED 1			//成功
#define FAILED -1			//失败
#define PD_PI 3.1415926		
#define TWOPI 6.2831852

#define IMGHEIGHT 81		//图片尺寸归一化高度
#define IMGWIDTH 108		//图片尺寸归一化宽度
#define SRCIMGHEIGHT 480	//原图高度
#define SRCIMGWIDTH 640		//原图宽度
#define GLINT_THRESHOLD 200			//排除光斑的灰度值的阈值
#define FRAME_THRESHOLD 10			//截取图片边框的宽度
#define A_B_THRESHOLD 5				//椭圆短轴b的最小长度
#define RATIO_THRESHOLD 0.4			//椭圆短轴与长轴之比的最小值
#define THETA_INCREMENT 0.05		//计算椭圆圆周灰度值的平均值时theta值的增量
#define ALPHA_INCREMENT 0.3			//遍历椭圆alpha角度值的增量
#define CENTER_THRESHOLD 100		//瞳孔中心灰度值阈值，大于该值则排除

using namespace std;  
using namespace cv; 

typedef struct		//椭圆
{
	int x;			//中心横坐标
	int y;			//中心纵坐标
	int a;			//长轴
	int b;			//短轴
	double alpha;		//偏转角度
}PDEllipse;

typedef struct		//4个方向的梯度值
{
	int x;			//x
	int y;			//y
//	int gradDir;	//计算梯度的方向标志
}PDPoint;

typedef struct		//梯度运算的三个点
{
	int outPointVal;
	int inPointVal;

}PDThreePoints;

class PupilDetection
{
public:
	//接口
	PupilDetection(Mat &_img, int _offset = 3);
	PupilDetection();
	~PupilDetection();
	/* 设置offset */
	void setOffset(int _offset){m_offset = _offset; return;}
	/* 传入图像 */
	int setImg(Mat &_img);
	/* 进行瞳孔检测定位，检测到的椭圆存入ellipse中 */
	int DetectPupil(void);
	/* 返回ellipse */
	void getEllipse(PDEllipse &_ellipse);
	/* 传出图像 */
	int getImg(Mat &_img);

private:
	/* 对传入图像进行处理 */
	int ImgProcessing(void);
	/* 计算图像上各点4个方向的梯度值 */
	void GradCalcu(void);
	/* 计算给定a, b, alpha下的椭圆圆周各点 */
	int PeripheryPointCalcu(const int a, const int b, const int alphaK, 
										 int &minX, int &maxX, int &minY, int &maxY);
	/* 计算给定椭圆及偏移量下的差分计算 */
	double EllipDiffCalcu(const int _x, const int _y, const int count);
	/* 计算椭圆边界像素点平均值 */
	double EllipseCalcu(const int _x, const int _y, const int count);
	/* 绘制椭圆 */
	void PDDrawEllipse(Mat &_img, int imgWidth, int imgHeight);
	/* 原图调整检测 */
	void SrcImgDetectPupil(void);
	double SrcImgEllipseCalcu(const int _x, const int _y, const int count);
	/* 计算给定a, b, alpha下的椭圆圆周各点 */
	int SrcImgPeripheryPointCalcu(const int a, const int b, const int alphaK,
										 int &minX, int &maxX, int &minY, int &maxY);
	/* 计算给定两点确定的直线 A * x + B * y + C = 0 */
	void LinearEquationCalcu(CvPoint &point1, CvPoint &point2, int &A, int &B, int &C);
	/* 对光斑区域进行填充,对 A*x + B*y + C < 0区域填充filledVal1 */
	void SpotAreaFill(Mat &img, CvPoint leftTopPoint, CvPoint rightBottomPoint, int filledVal1, int filledVal2,
		int A, int B, int C);
	/* 瞳孔粗定位 */
	void PupilCoarsePositioning();
	/* 进行瞳孔粗定位时轮廓的筛选 */
	bool ContourScreening(vector<Point> &contours, double &valueAccum, Point &circleCenter, int &circleRadius);
	void PDDrawCircle(Mat &img, int r, int _x, int _y);

	//数据变量
	int m_offset;				//椭圆差分计算偏移量
	int m_srcImgOffset;			//原图椭圆差分计算偏移量
	PDEllipse m_ellipse;		//椭圆
	Mat m_img;					//图像
	Mat m_srcImg;
	int m_minX;
	int m_maxX;
	int m_minY;
	int m_maxY;

	//中间变量
	int m_peripheryPoints[2000];
	PDThreePoints m_srcPeripheryPoints[2000];
	int m_srcImgWidth;
	int m_srcImgHeight;
	int m_grads[IMGHEIGHT * IMGWIDTH];
	double *m_cosTheta;	//存放cos theta的值的数组
	double *m_sinTheta;	//存放sin theta的值的数组
	double *m_cosAlpha;	//存放cos alpha的值的数组
	double *m_sinAlpha;	//存放sin alpha的值的数组
};


#endif