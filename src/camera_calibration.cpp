// TEST4_CLB.cpp : 定义控制台应用程序的入口点。
#include <iostream> 
#include <math.h>
#include <boost/lexical_cast.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"  
#include "opencv2/imgproc/imgproc.hpp" 

using namespace cv;
using namespace std;

# define PI 3.141592654
int imageWidth = 1920;                             //摄像头的分辨率  
int imageHeight = 1080;

int boardWidth = 9;                               //横向的角点数目  
int boardHeight = 6;                              //纵向的角点数据  
int boardCorner = boardWidth * boardHeight;       //总的角点数据  
int frameNumber = 13;                             //相机标定时需要采用的图像帧数  
int squareSize = 10;                              //标定板黑白格子的大小 单位mm  
Size boardSize = Size(boardWidth, boardHeight);   //  

Mat intrinsic;                                          //相机内参数  
Mat distortion_coeff;                                   //相机畸变参数  
vector<Mat> rvecs;                                        //旋转向量  
vector<Mat> tvecs;                                        //平移向量  
vector<vector<Point2f>> corners;                        //各个图像找到的角点的集合 和objRealPoint 一一对应  
vector<vector<Point3f>> objRealPoint;                   //各副图像的角点的实际物理坐标集合  
vector<Point2f> corner;                                   //某一副图像找到的角点  

Mat rgbImage, grayImage;
void calRealPoint(vector<vector<Point3f>>& obj, int boardwidth, int boardheight, int imgNumber, int squaresize)
{
	//  Mat imgpoint(boardheight, boardwidth, CV_32FC3,Scalar(0,0,0));  
	vector<Point3f> imgpoint;
	for (int rowIndex = 0; rowIndex < boardheight; rowIndex++)
	{
		for (int colIndex = 0; colIndex < boardwidth; colIndex++)
		{
			//  imgpoint.at<Vec3f>(rowIndex, colIndex) = Vec3f(rowIndex * squaresize, colIndex*squaresize, 0);  
			imgpoint.push_back(Point3f(rowIndex * squaresize, colIndex * squaresize, 0));
		}
	}
	for (int imgIndex = 0; imgIndex < imgNumber; imgIndex++)
	{
		obj.push_back(imgpoint);
	}
}

void outputCameraParam(void)
{
	/*保存数据*/
	//cvSave("cameraMatrix.xml", &intrinsic);  
	//cvSave("cameraDistoration.xml", &distortion_coeff);  
	//cvSave("rotatoVector.xml", &rvecs);  
	//cvSave("translationVector.xml", &tvecs);  
	/*输出数据*/
	cout<<"cameraMatrix:\n \
	fx 0 cx \n \
	0 fy cy \n \
	0 0  1\n"<<endl;
	cout << "fx :" << intrinsic.at<double>(0, 0) << endl << "fy :" << intrinsic.at<double>(1, 1) << endl;
	cout << "cx :" << intrinsic.at<double>(0, 2) << endl << "cy :" << intrinsic.at<double>(1, 2) << endl;
	cout<<"\ncameraDistoration:"<<endl;
	cout << "k1 :" << distortion_coeff.at<double>(0, 0) << endl;
	cout << "k2 :" << distortion_coeff.at<double>(1, 0) << endl;
	cout << "p1 :" << distortion_coeff.at<double>(2, 0) << endl;
	cout << "p2 :" << distortion_coeff.at<double>(3, 0) << endl;
	cout << "k3 :" << distortion_coeff.at<double>(4, 0) << endl;
	cout<<"\n视场角FOV:"<<endl;
	double FOV_X = 2*atan(imageWidth/(2*intrinsic.at<double>(0, 0)));
	double FOV_y = 2*atan(imageHeight/(2*intrinsic.at<double>(1, 1)));
	cout <<"水平视场角(弧度):"<<FOV_X<<endl;//"水平视场角:"
	cout <<"垂直视场角(弧度):"<<FOV_y<<endl;//"垂直视场角:"
}
void guessCameraParam(void)
{
	/*分配内存*/
	intrinsic.create(3, 3, CV_64FC1);
	distortion_coeff.create(5, 1, CV_64FC1);

	/*
	fx 0 cx
	0 fy cy
	0 0  1
	*/
	intrinsic.at<double>(0, 0) = 700;   //fx         
	intrinsic.at<double>(0, 2) = 700;   //cx  
	intrinsic.at<double>(1, 1) = 500;   //fy  
	intrinsic.at<double>(1, 2) = 500;   //cy  

	intrinsic.at<double>(0, 1) = 0;
	intrinsic.at<double>(1, 0) = 0;
	intrinsic.at<double>(2, 0) = 0;
	intrinsic.at<double>(2, 1) = 0;
	intrinsic.at<double>(2, 2) = 1;

	/*
	k1 k2 p1 p2 k3
	*/
	distortion_coeff.at<double>(0, 0) = -0.463286;    //k1  
	distortion_coeff.at<double>(1, 0) = 0.37891;      //k2  
	distortion_coeff.at<double>(2, 0) = -0.00164795;  //p1  
	distortion_coeff.at<double>(3, 0) = -0.000847324; //p2  
	distortion_coeff.at<double>(4, 0) = -0.30986;     //k3  
}



int main(int argc, char* argv[])
{
	Mat img;
	int goodFrameCount = 0;	
	if(argc!=5)
	{
		printf("please enter the (boardWidth boardHeight squareSize frameNumber) after the main function\n");
		return 0;
	}
	try
	{
		boardWidth = boost::lexical_cast<int>(argv[1]);
		boardHeight = boost::lexical_cast<int>(argv[2]);
		squareSize = boost::lexical_cast<int>(argv[3]);
		frameNumber = boost::lexical_cast<int>(argv[4]);
		boardCorner = boardWidth * boardHeight;       //总的角点数据  
		boardSize = Size(boardWidth, boardHeight);  //
	}catch(const boost::bad_lexical_cast &e)
	{
		printf("please enter the (boardWidth boardHeight squareSize frameNumber) after the main function\n");
		printf("%s\r\n",e.what());
		printf("error:the number of photo fail!\n");
		return 0;
	}
	namedWindow("chessboard");
	cout << "press Q to quit ..." << endl;
	while (goodFrameCount < frameNumber)
	{
		char filename[100];
		snprintf(filename,sizeof(filename), "%d.jpg", goodFrameCount + 1);
		cout << filename << endl;
		rgbImage = imread(filename, 1);
		//grayImage = imread(filename, 0);		
		imageHeight = rgbImage.rows;
		imageWidth = rgbImage.cols;
		//cout<<"width"<<imageWidth<<endl;
		//cout<<"height"<<imageHeight<<endl;
		cvtColor(rgbImage, grayImage, CV_BGR2GRAY);
		//imshow("Camera", grayImage);

		bool isFind = findChessboardCorners(rgbImage, boardSize, corner, 0);
		if (isFind == true) //所有角点都被找到 说明这幅图像是可行的  
		{

			/*
			Size(5,5) 搜索窗口的一半大小
			Size(-1,-1) 死区的一半尺寸
			TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1)迭代终止条件
			*/
			cornerSubPix(grayImage, corner, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.01));
			//cout << 1;
			drawChessboardCorners(rgbImage, boardSize, corner, isFind);
			imshow("chessboard", rgbImage);
			
			int c = waitKey(15);
			if (c == 'p')
			{
				c = 0;
				while (c != 'p'&& c != 27)
				{
					c = waitKey(50);
				}
			}
			if (c == 27)
				break;

			//cout << goodFrameCount << endl;
			//cout << corner << endl;
			corners.push_back(corner);
			//string filename = "res\\image\\calibration";  
			//filename += goodFrameCount + ".jpg";  
			//cvSaveImage(filename.c_str(), &IplImage(rgbImage));       //把合格的图片保存起来  
			goodFrameCount++;

			cout << "The image is good" << endl;
		}
		else
		{
			cout << "The image is bad please try again" << endl;
		}
		if (waitKey(10) == 'q')
		{
			break;
		}
	}
	guessCameraParam();
	cout << "guess successful" << endl;
	/*计算实际的校正点的三维坐标*/
	
	calRealPoint(objRealPoint, boardWidth, boardHeight, frameNumber, squareSize);
	cout << "cal real successful" << endl;
	/*标定摄像头*/
	//Size(imageWidth, imageHeight)
	calibrateCamera(objRealPoint, corners, rgbImage.size(),intrinsic, distortion_coeff, rvecs, tvecs, 0);
	cout << "calibration successful\n" << endl;
	/*保存并输出参数*/
	
	outputCameraParam();

	/*cout << "out successful" << endl;
	Mat mapx = Mat(rgbImage.size(), CV_32FC1);
	Mat mapy = Mat(rgbImage.size(), CV_32FC1);
	Mat R = Mat::eye(3, 3, CV_32F);
	initUndistortRectifyMap(intrinsic, distortion_coeff, R,
		intrinsic,rgbImage.size(), CV_32FC1, mapx, mapy);*/
	/*显示畸变校正效果*/
	//Mat camera;
	//Mat cImage = rgbImage.clone();
	//while (!rgbImage.empty())
	//{
	//	capture>>rgbImage;
	//	//rgbImage = imread("test2.jpg");
	//	//undistort(rgbImage, cImage, intrinsic, distortion_coeff);
	//	remap(rgbImage, cImage, mapx, mapy, INTER_LINEAR);
	//	imshow("undistort Image", cImage);
	//	//imwrite("undistort_Image.jpg",cImage);
	//	//cout << "Correct Image" << endl;
	//	int c = waitKey(15);
	//	if (c == 'p')
	//	{
	//		c = 0;
	//		while (c != 'p'&& c != 27)
	//		{
	//			c = waitKey(50);
	//		}
	//	}
	//	if (c == 27)
	//		break;
	//}
	cvDestroyAllWindows();
	return 0;
}

