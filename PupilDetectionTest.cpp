
#include "PupilDetection.h"

/* ��PDEllipse������ת����string */
void ConvertTostring(PDEllipse &srcEllipse, vector<string> &vecStr);

/* ���������в��� */

int PupilDetectionTest(void)
{
	/* ���� */
	/************************************************************************/

	//��ȡͼ��·��
	string testImgPath = "E:/OpenCVFile/PupilDetection/src/eyeImages/eye_1/";
//	string testImgPath = "E:/OpenCVFile/PupilDetection/src/eyeImages/imgs/";	//����
//	string testImgPath = "./srcImg/1/";				
	string extend = "*.bmp";		//ͼ������

	//����ͼƬ·����"eye_" Ϊ�ļ���ǰ׺
	string preName = "./images/2/eye_";
//	string preName = "./dstImg/1/eye_";			//��װ
	//������Խ������·��
	string resultFileName = "resultFile3.txt";


	int imgNum = 200;			//���ļ���������ͼ�������

	/************************************************************************/

	clock_t start, end, tempStart, tempEnd;
	Mat img;
	vector<Mat> vecReadMats;
	vector<Mat> vecWriteMats;
	Directory dir; 
	//��ȡͼ��
	bool imgFind = false;
	vector<string> imgPath = dir.GetListFiles(testImgPath, extend, imgFind);

	PupilDetection pulDetec;
	pulDetec.setOffset(3);			//����offset
	vector<PDEllipse> vecEllipse;
	PDEllipse resultEllipse;

	if (imgNum > imgPath.size())
	{
		imgNum = imgPath.size();
	}
	//��ͼ�����vecReadMats��
	for (int k = 0; k < imgNum; k ++)
	{
		string imgName = imgPath[k];
		string imgFullName = testImgPath + imgName;
		img = imread(imgFullName);
		vecReadMats.push_back(img);
	}

	cout << "��ʼ���ԣ�" << endl << endl;
	start = clock();
	for (int i = 0; i < imgNum; i ++)
	{
/*		cout << "�� " << i << " ��ͼ : " << imgPath[i] << endl;
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

	cout << "����ͼƬΪ��" << imgNum << "	������ʱ��Ϊ��" << end - start << " ms" << endl;
	cout << "ƽ��ÿ��ͼƬ����ʱ��Ϊ��" << (end - start) / imgNum  << " ms" << endl;

	//����ͼƬ������
	string fullName;
	stringstream ss;
	vector<string> vecStr;
	ofstream resultFile(resultFileName);
	if (!resultFile)
	{
#if _DEBUG
		cout << "���ļ� " << resultFileName << " ʧ��!" << endl;
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

	cout << endl << "�������ԣ�" << endl;
	return SUCCEED;

}


/* ��PDEllipse������ת����string */
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
