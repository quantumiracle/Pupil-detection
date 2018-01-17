
#include "PupilDetection.h"

/* 将PDEllipse各个数转换成string */
void ConvertTostring(PDEllipse &srcEllipse, vector<string> &vecStr);

/* 对样本进行测试 */

int PupilDetectionTest(void)
{
	/* 参数 */
	/************************************************************************/

	//读取图像路径
	string testImgPath = "E:/OpenCVFile/PupilDetection/src/eyeImages/eye_1/";
//	string testImgPath = "E:/OpenCVFile/PupilDetection/src/eyeImages/imgs/";	//调试
//	string testImgPath = "./srcImg/1/";				
	string extend = "*.bmp";		//图像类型

	//保存图片路径，"eye_" 为文件名前缀
	string preName = "./images/2/eye_";
//	string preName = "./dstImg/1/eye_";			//封装
	//保存测试结果数据路径
	string resultFileName = "resultFile3.txt";


	int imgNum = 200;			//从文件中最多测试图像的数量

	/************************************************************************/

	clock_t start, end, tempStart, tempEnd;
	Mat img;
	vector<Mat> vecReadMats;
	vector<Mat> vecWriteMats;
	Directory dir; 
	//读取图像
	bool imgFind = false;
	vector<string> imgPath = dir.GetListFiles(testImgPath, extend, imgFind);

	PupilDetection pulDetec;
	pulDetec.setOffset(3);			//设置offset
	vector<PDEllipse> vecEllipse;
	PDEllipse resultEllipse;

	if (imgNum > imgPath.size())
	{
		imgNum = imgPath.size();
	}
	//将图像放入vecReadMats中
	for (int k = 0; k < imgNum; k ++)
	{
		string imgName = imgPath[k];
		string imgFullName = testImgPath + imgName;
		img = imread(imgFullName);
		vecReadMats.push_back(img);
	}

	cout << "开始测试！" << endl << endl;
	start = clock();
	for (int i = 0; i < imgNum; i ++)
	{
/*		cout << "第 " << i << " 张图 : " << imgPath[i] << endl;
		if (32 == i)
		{
			i ++;
			i --;
		}
*/
		img = vecReadMats[i];
//		tempStart = clock();
		
		pulDetec.setImg(img);
		pulDetec.DetectPupil();
		pulDetec.getImg(img);
		pulDetec.getEllipse(resultEllipse);

		vecWriteMats.push_back(img);
		vecEllipse.push_back(resultEllipse);
//		tempEnd = clock();
//		cout << i << " : " << tempEnd - tempStart << " ms" << endl;
		
	}

	end = clock();

	cout << "测试图片为：" << imgNum << "	总运行时间为：" << end - start << " ms" << endl;
	cout << "平均每张图片运行时间为：" << (end - start) / imgNum  << " ms" << endl;

	//保存图片及数据
	string fullName;
	stringstream ss;
	vector<string> vecStr;
	ofstream resultFile(resultFileName);
	if (!resultFile)
	{
#if _DEBUG
		cout << "打开文件 " << resultFileName << " 失败!" << endl;
#endif
		exit(0);
	}

	for (int j = 0; j < vecWriteMats.size(); j ++)
	{
//		resize(vecWriteMats[j], vecWriteMats[j], Size(360, 270));
		ss.str("");
		ss << j;
		fullName = preName + ss.str() + ".bmp";
		imwrite(fullName, vecWriteMats[j]);
		vecStr.clear();
		ConvertTostring(vecEllipse[j], vecStr);
		resultFile << "x: " << vecStr[0] << "  y: " << vecStr[1] << "  a: " << vecStr[2]
		<< "  b: " << vecStr[3] << "  alpha: " << vecStr[4] << "  " << fullName << '\t' << imgPath[j] << endl;

	}
	
	resultFile.close();

	cout << endl << "结束测试！" << endl;
	return SUCCEED;

}


/* 将PDEllipse各个数转换成string */
void ConvertTostring(PDEllipse &srcEllipse, vector<string> &vecStr)
{
	stringstream ss;
	string str;
	
	ss.str("");
	ss << srcEllipse.x;
	str = ss.str();
	vecStr.push_back(str);

	ss.str("");
	ss << srcEllipse.y;
	str = ss.str();
	vecStr.push_back(str);

	ss.str("");
	ss << srcEllipse.a;
	str = ss.str();
	vecStr.push_back(str);

	ss.str("");
	ss << srcEllipse.b;
	str = ss.str();
	vecStr.push_back(str);

	ss.str("");
	ss << srcEllipse.alpha;
	str = ss.str();
	vecStr.push_back(str);
	return;

}
