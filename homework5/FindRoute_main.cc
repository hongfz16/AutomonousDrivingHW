#include"FindRoute.h"

int main()
{
	for(int i=1;i<=5;++i)
	{
		//PLEASE USE THE RIGHT PATH
		string inputfilename="/home/hongfz/Documents/Learn/AutomonousDrivingHW/homework5/data/routes/route_request_";
		string outputfilename="/home/hongfz/Documents/Learn/AutomonousDrivingHW/homework5/route_result_";
		inputfilename+=to_string(i)+string(".txt");
		outputfilename+=to_string(i)+string(".txt");
		FindRoute(inputfilename,outputfilename);
	}
	return 0;
}