#include "LaneDetect.h"

void LaneDetect(const Mat& srcimgori,Mat& _result)
{
	Rect roi(0,srcimgori.cols/3,srcimgori.cols-1,srcimgori.rows - srcimgori.cols/3);
	Mat srcimg=srcimgori(roi);

	int houghVote=200;
	double PI=3.141592653;

	Mat whiteMask,yellowMask,mask,masked,hlsConvert;
	cvtColor(srcimg,hlsConvert,COLOR_RGB2HLS);
	//imshow("hlsConvert",hlsConvert);
	inRange(hlsConvert,Scalar(0,200,0),Scalar(255,255,255),whiteMask);
	inRange(hlsConvert,Scalar(100,0,10),Scalar(255,255,255),yellowMask);
	bitwise_or(whiteMask,yellowMask,mask);
	bitwise_and(srcimg,srcimg,masked,mask);

	//imshow("masked",masked);

	Mat grayMasked,blurGrayMasked;
	cvtColor(masked,grayMasked,CV_RGB2GRAY);
	GaussianBlur(grayMasked,blurGrayMasked,Size(15,15),0);

	//imshow("blurGrayMasked",blurGrayMasked);
	Mat cannyed;
	Canny(blurGrayMasked,cannyed,50,100);
	//imshow("cannyed",cannyed);

	vector<Vec2f> lines;
	while(lines.size() < 5 && houghVote > 0)
	{
		HoughLines(cannyed,lines,1,PI/180, houghVote);
		houghVote -= 5;
	}
	Mat result(srcimg.size(),CV_8U,Scalar(0));
	srcimgori.copyTo(result);

	auto it=lines.begin();
	Mat hough(srcimg.size(),CV_8U,Scalar(0));
	while(it!=lines.end())
	{
		float rho=(*it)[0];
		float theta=(*it)[1];
		if (theta > 0.09 && theta < 1.48 || theta < 3.14 && theta > 1.66 )
		{ // filter to remove vertical and horizontal lines
		
			// point of intersection of the line with first row
			Point pt1(rho/cos(theta),0+srcimgori.cols/3);        
			// point of intersection of the line with last row
			Point pt2((rho-result.rows*sin(theta))/cos(theta),result.rows+srcimgori.cols/3);
			// draw a white line
			line( result, pt1, pt2, Scalar(0,0,255), 2); 
			line( hough, pt1, pt2, Scalar(0,0,255), 2);
		}
		++it;
	}

	imshow("Result",result);

	_result=result;
}