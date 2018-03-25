#include "LaneDetect.h"

inline double size(Vec4d& line)
{
	return sqrt(pow(line[0]-line[2],2)+pow(line[1]-line[3],2));
}

inline double slope(Vec4d& line)
{
	return (line[3]-line[1])/(line[2]-line[0]);
}

bool cmpsize(Vec4d& line1,Vec4d& line2)
{
	return size(line1)>size(line2);
}

void DrawLines(Mat& result,vector<Vec4d>& lines)
{
	vector<Vec4d> rightlines,leftlines;
	for(int i=0;i<lines.size();++i)
	{
		double theta=(lines[i][3]-lines[i][1])/(lines[i][2]-lines[i][0]);
		//cout<<theta<<endl;
		if(theta<-0.5)
		{
			rightlines.push_back(lines[i]);
		}
		else if(theta>0.5)
		{
			leftlines.push_back(lines[i]);
		}
	}
	sort(rightlines.begin(),rightlines.end(),cmpsize);
	sort(leftlines.begin(),leftlines.end(),cmpsize);

	double left_avg_slope=0,right_avg_slope=0;

	for(int i=0;i<6 && i<leftlines.size() && i<rightlines.size();++i)
	{
		left_avg_slope+=slope(leftlines[i]);
		right_avg_slope+=slope(rightlines[i]);
	}
	left_avg_slope/=6;
	right_avg_slope/=6;

	double left_y_intercept_avg=0,right_y_intercept_avg=0;

	//vector<double> left_intercept,right_intercept;
	for(int i=0;i<6 && i<leftlines.size();++i)
	{
		double tmp1=leftlines[i][1]-left_avg_slope*leftlines[i][0];
		double tmp2=leftlines[i][3]-left_avg_slope*leftlines[i][2];
		//left_intercept.push_back(tmp1);
		//left_intercept.push_back(tmp2);
		left_y_intercept_avg+=(tmp1+tmp2);
	}
	
	for(int i=0;i<6 && i<rightlines.size();++i)
	{
		double tmp1=rightlines[i][1]-right_avg_slope*rightlines[i][0];
		double tmp2=rightlines[i][3]-right_avg_slope*rightlines[i][2];
		//right_intercept.push_back(tmp1);
		//right_intercept.push_back(tmp2);
		right_y_intercept_avg+=(tmp1+tmp2);
	}

	left_y_intercept_avg/=12;
	right_y_intercept_avg/=12;

	double imgfloor=result.rows;
	double horizonline=imgfloor/1.5;

	double max_left_x=(horizonline-left_y_intercept_avg)/left_avg_slope;
	double min_left_x=(imgfloor-left_y_intercept_avg)/left_avg_slope;

	double max_right_x=(imgfloor-right_y_intercept_avg)/right_avg_slope;
	double min_right_x=(horizonline-right_y_intercept_avg)/right_avg_slope;

	Point l1((int)(min_left_x),(int)(imgfloor));
	Point l2((int)(max_left_x),(int)(horizonline));
	Point r1((int)(max_right_x),(int)(imgfloor));
	Point r2((int)(min_right_x),(int)(horizonline));

	line(result,l1,l2,Scalar(0,0,255),4);
	line(result,r1,r2,Scalar(255,0,0),4);
}

void LaneDetect(const Mat& srcimgori,Mat& _result)
{
	Rect roi(0,srcimgori.cols/3,srcimgori.cols-1,srcimgori.rows - srcimgori.cols/3);
	Mat srcimg=srcimgori(roi);

	//int houghVote=200;
	double PI=3.141592653;

	Mat whiteMask,yellowMask,mask,masked,hlsConvert;
	cvtColor(srcimg,hlsConvert,COLOR_RGB2HLS);
	//imshow("hlsConvert",hlsConvert);
	inRange(hlsConvert,Scalar(0,200,0),Scalar(255,255,255),whiteMask);
	//inRange(hlsConvert,Scalar(100,0,10),Scalar(255,255,255),yellowMask);
	//bitwise_or(whiteMask,yellowMask,mask);
	bitwise_and(srcimg,srcimg,masked,whiteMask);

	//imshow("masked",masked);

	Mat grayMasked,blurGrayMasked;
	cvtColor(masked,grayMasked,CV_RGB2GRAY);
	GaussianBlur(grayMasked,blurGrayMasked,Size(15,15),0);

	//imshow("blurGrayMasked",blurGrayMasked);
	Mat cannyed;
	Canny(blurGrayMasked,cannyed,50,150);
	//imshow("cannyed",cannyed);

	vector<Vec4d> lines;
	//while(lines.size() < 5 && houghVote > 0)
	//{
	HoughLinesP(cannyed,lines,2,PI/180,5,10,20);
	//	houghVote -= 5;
	//}
	Mat result(srcimg.size(),CV_8U,Scalar(0));
	srcimgori.copyTo(result);

	//auto it=lines.begin();
	//Mat hough(srcimg.size(),CV_8U,Scalar(0));
	
	// while(it!=lines.end())
	// {
	// 	float rho=(*it)[0];
	// 	float theta=(*it)[1];
	// 	if (theta > 0.09 && theta < 1.48 || theta < 3.14 && theta > 1.66 )
	// 	{ // filter to remove vertical and horizontal lines
		
	// 		// point of intersection of the line with first row
	// 		Point pt1(rho/cos(theta),0+srcimgori.cols/3);        
	// 		// point of intersection of the line with last row
	// 		Point pt2((rho-result.rows*sin(theta))/cos(theta),result.rows+srcimgori.cols/3);
	// 		// draw a white line
	// 		line( result, pt1, pt2, Scalar(0,0,255), 2); 
	// 		line( hough, pt1, pt2, Scalar(0,0,255), 2);
	// 	}
	// 	++it;
	// }

	DrawLines(result,lines);

	imshow("Result",result);

	_result=result;
}