#include "DrawHistogram.h"

void DrawHistHight(const vector<Eigen::Vector3d>& points)
{
	Mat data(1,points.size(),CV_32F);
	float* pdata=data.ptr<float>();
	float max=-1000000;
	float min=10000000;
	for(size_t i=0;i<points.size();++i)
	{
		pdata[i]=points[i](2);
		if(pdata[i]>max)
			max=pdata[i];
		if(pdata[i]<min)
			min=pdata[i];
	}
	
	Mat h_hist;
	int histSize=100;
	float range[2];
	range[0]=min;
	range[1]=max;
	cout<<min<<" "<<max<<endl;
	const float* histRange={range};
	bool uniform=true;
	bool accumulate=false;
	calcHist(&data,1,0,Mat(),h_hist,1,&histSize,&histRange,uniform,accumulate);
	
	int hist_w=1024;
	int hist_h=900;
	int bin_w=cvRound((double)hist_w/histSize);
	Mat histImage(hist_h,hist_w,CV_8UC3,Scalar(0,0,0));
	
	normalize(h_hist,h_hist,0,histImage.rows,NORM_MINMAX,-1,Mat());
	for(int i=1;i<histSize;i++)
	{
		line(histImage,Point(bin_w*(i-1),hist_h-cvRound(h_hist.at<float>(i-1))),
			Point(bin_w*(i),hist_h-cvRound(h_hist.at<float>(i))),
			Scalar(255,0,0),2,8,0);
	}

	namedWindow("Histogram For Hight",CV_WINDOW_AUTOSIZE);
	imshow("Histogram For Hight",histImage);
}

void DrawHistDist(const vector<Eigen::Vector3d>& points)
{
	Mat data(1,points.size(),CV_32F);
	float* pdata=data.ptr<float>();
	float max=-1000000;
	float min=10000000;
	for(size_t i=0;i<points.size();++i)
	{
		pdata[i]=sqrt(points[i].transpose()*points[i]);
		if(pdata[i]>max)
			max=pdata[i];
		if(pdata[i]<min)
			min=pdata[i];
	}
	
	Mat h_hist;
	int histSize=100;
	float range[2];
	range[0]=min;
	range[1]=max;
	const float* histRange={range};
	bool uniform=true;
	bool accumulate=false;
	calcHist(&data,1,0,Mat(),h_hist,1,&histSize,&histRange,uniform,accumulate);
	
	int hist_w=1024;
	int hist_h=900;
	int bin_w=cvRound((double)hist_w/histSize);
	Mat histImage(hist_h,hist_w,CV_8UC3,Scalar(0,0,0));
	
	normalize(h_hist,h_hist,0,histImage.rows,NORM_MINMAX,-1,Mat());
	for(int i=1;i<histSize;i++)
	{
		line(histImage,Point(bin_w*(i-1),hist_h-cvRound(h_hist.at<float>(i-1))),
			Point(bin_w*(i),hist_h-cvRound(h_hist.at<float>(i))),
			Scalar(255,0,0),2,8,0);
	}

	namedWindow("Histogram For Length",CV_WINDOW_AUTOSIZE);
	imshow("Histogram For Length",histImage);
}