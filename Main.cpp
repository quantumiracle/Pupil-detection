
#include "PupilDetection.h"

/* 对样本进行测试 */
int PupilDetectionTest(void);
/* 用于调试 */
void DebugTest(void);

//int GlintProcessing(Mat &srcImg);

int main()
{
	//用于调试
//	DebugTest();		
	//对样本进行测试
	PupilDetectionTest();

//	Mat img = imread("eye_1#120.bmp");
//	GlintProcessing(img);
	system("pause");
	return 1;
}
