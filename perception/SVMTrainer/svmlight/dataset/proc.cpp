#include <iostream>
#include <string>
#include <fstream>

using namespace std;

int main()
{
	ifstream fin("train.txt");
	ofstream fout("train_proc.txt");
	string line;
	string nan("nan");
	while(getline(fin,line))
	{
		if(line.find(nan)==std::string::npos)
			fout<<line<<endl;
	}
	return 0;
}