#include "LaneDetect.h"

void LaneDetect(const Mat& srcimgori)
{
	// Mat gray;
	// cvtColor(srcimg,gray,CV_RGB2GRAY);
	// //imshow("gray",gray);
	Rect roi(0,srcimgori.cols/3,srcimgori.cols-1,srcimgori.rows - srcimgori.cols/3);
	// Mat imgROI = gray(roi);
	// //imshow("imgROI",imgROI);
	// Mat contours;
	// Canny(imgROI,contours,50,250);
	// Mat contoursInv;
	// threshold(contours,contoursInv,128,255,THRESH_BINARY_INV);
	// //imshow("Contours1",contoursInv);
	Mat srcimg=srcimgori(roi);

	int houghVote=200;
	double PI=3.141592653;
	// vector<Vec2f> lines;
	// while(lines.size() < 5 && houghVote > 0)
	// {
	// 	HoughLines(contours,lines,1,PI/180, houghVote);
	// 	houghVote -= 5;
	// }
	// Mat result(imgROI.size(),CV_8U,Scalar(0));
	// imgROI.copyTo(result);

	// auto it=lines.begin();
	// Mat hough(imgROI.size(),CV_8U,Scalar(0));
	// while(it!=lines.end())
	// {
	// 	float rho=(*it)[0];
	// 	float theta=(*it)[1];
	// 	if ( theta > 0.09 && theta < 1.48 || theta < 3.14 && theta > 1.66 )
	// 	{ // filter to remove vertical and horizontal lines
		
	// 		// point of intersection of the line with first row
	// 		Point pt1(rho/cos(theta),0);        
	// 		// point of intersection of the line with last row
	// 		Point pt2((rho-result.rows*sin(theta))/cos(theta),result.rows);
	// 		// draw a white line
	// 		line( result, pt1, pt2, Scalar(255), 8); 
	// 		line( hough, pt1, pt2, Scalar(255), 8);
	// 	}

	// 	//std::cout << "line: (" << rho << "," << theta << ")\n"; 
	// 	++it;
	// }
	// imshow("result",result);
	//imshow("src",srcimg);

	Mat whiteMask,yellowMask,mask,masked,hlsConvert;
	cvtColor(srcimg,hlsConvert,COLOR_RGB2HLS);
	inRange(hlsConvert,Scalar(0,200,0),Scalar(255,255,255),whiteMask);
	inRange(hlsConvert,Scalar(100,0,10),Scalar(255,255,255),yellowMask);
	bitwise_or(whiteMask,yellowMask,mask);
	bitwise_and(srcimg,srcimg,masked,mask);

	Mat grayMasked,blurGrayMasked;
	cvtColor(masked,grayMasked,CV_RGB2GRAY);
	GaussianBlur(grayMasked,blurGrayMasked,Size(15,15),0);

	Mat cannyed;
	Canny(blurGrayMasked,cannyed,50,100);

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
		if ( theta > 0.09 && theta < 1.48 || theta < 3.14 && theta > 1.66 )
		{ // filter to remove vertical and horizontal lines
		
			// point of intersection of the line with first row
			Point pt1(rho/cos(theta),0+srcimgori.cols/3);        
			// point of intersection of the line with last row
			Point pt2((rho-result.rows*sin(theta))/cos(theta),result.rows+srcimgori.cols/3);
			// draw a white line
			line( result, pt1, pt2, Scalar(0,0,255), 5); 
			line( hough, pt1, pt2, Scalar(0,0,255), 8);
		}
		++it;
	}

	imshow("mask",blurGrayMasked);
}