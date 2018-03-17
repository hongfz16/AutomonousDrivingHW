#include "RGB2Grey.h"

void RGB2Grey(const Mat& src,Mat& re)
{
	re.create(src.size(),CV_8UC1);

	int channels=src.channels();
	for(int i=0;i<re.rows;++i)
	{
		uchar* reline=re.ptr<uchar>(i);
		const uchar* srcline=src.ptr<uchar>(i);
		for(int j=0;j<re.cols;++j)
		{
			int R=srcline[j*channels];
			int G=srcline[j*channels+1];
			int B=srcline[j*channels+2];
			reline[j]=(R*30 + G*59 + B*11 + 50) / 100;
		}
	}
}
