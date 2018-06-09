#include "FeatureExtract.h"

RTree<int,double,3> rt;

void BasicStatisticFeatures(const vector<Eigen::Vector3d>& points, vector<double>& re)
{
	double f1=0,f2=0,f3=0,f4=0,f5=0,f6=0,
		   x_mean=0,y_mean=0,z_mean=0,
		   t1=0,t2=0,t3=0,t4=0,t5=0,t6=0;
	for(int i=0;i<points.size();++i)
	{
		x_mean+=points[i](0);
		y_mean+=points[i](1);
		z_mean+=points[i](2);
	}


	x_mean/=points.size();
	y_mean/=points.size();
	z_mean/=points.size();

	for(int i=0;i<points.size();++i)
	{
		f1+=pow(points[i](0)-x_mean,2)+pow(points[i](1)-y_mean,2);
		f2+=pow(points[i](0)-x_mean,2)+pow(points[i](2)-z_mean,2);
		f3+=pow(points[i](1)-y_mean,2)+pow(points[i](2)-z_mean,2);
		f4+=-points[i](0)*points[i](1);
		f5+=-points[i](0)*points[i](2);
		f6+=-points[i](1)*points[i](2);

		t1+=(points[i](0)-x_mean)*(points[i](0)-x_mean);
		t2+=(points[i](1)-y_mean)*(points[i](1)-y_mean);
		t3+=(points[i](2)-z_mean)*(points[i](2)-z_mean);
		t4+=(points[i](0)-x_mean)*(points[i](1)-y_mean);
		t5+=(points[i](0)-x_mean)*(points[i](2)-z_mean);
		t6+=(points[i](1)-y_mean)*(points[i](2)-z_mean);
	}
	double sm=sqrt(pow(f1,2)+pow(f2,2)+pow(f3,2)+
				   pow(f4,2)+pow(f5,2)+pow(f6,2)+
				   pow(points.size(),2));
	
	re.push_back(points.size()/sm);
	re.push_back(f1/sm);
	re.push_back(f2/sm);
	re.push_back(f3/sm);
	re.push_back(f4/sm);
	re.push_back(f5/sm);
	re.push_back(f6/sm);

	re.push_back(t1/=(points.size()-1));
	re.push_back(t2/=(points.size()-1));
	re.push_back(t3/=(points.size()-1));
	re.push_back(t4/=(points.size()-1));
	re.push_back(t5/=(points.size()-1));
	re.push_back(t6/=(points.size()-1));

	Eigen::Matrix<double,3,3> cov_matrix;
	cov_matrix<<t1,t4,t5,
				t4,t2,t6,
				t5,t6,t3;
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(cov_matrix);
	if(eigen_solver.info()==Eigen::Success)
	{
		vector<double> ans;
		ans.push_back(eigen_solver.eigenvalues()(0));
		ans.push_back(eigen_solver.eigenvalues()(1));
		ans.push_back(eigen_solver.eigenvalues()(2));
		sort(ans.begin(),ans.end(),[](const double& l, const double& r){return l<r;});
		double sum=ans[0]+ans[1]+ans[2];
		for(int i=0;i<ans.size();++i)
		{
			ans[i]/=sum;
		}
		re.push_back(ans[2]);
		re.push_back(ans[2]-ans[1]);
		re.push_back(ans[1]-ans[0]);
	}
}

bool QueryResultCallback(int a_data, vector<int>& resultid)
{
	resultid.push_back(a_data);
	return true;
}

double dist(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2)
{
	return sqrt(pow(p1(0)-p2(0),2)+
				pow(p1(1)-p2(1),2)+
				pow(p1(2)-p2(2),2));
}

vector<double> divide_bins(vector<double>& l, int bins)
{
	vector<double> re;
	double max=-1e8;
	double min=1e8;
	for(int i=0;i<l.size();++i)
	{
		if(l[i]<min)
			min=l[i];
		if(l[i]>max)
			max=l[i];
	}
	double width=(max-min)/bins;
	re.resize(bins,0);
	if(abs(width)<1e-6)
	{
		re[0]=1;
		return re;	
	}
	for(int i=0;i<l.size();++i)
	{
		int slot=(l[i]-min)/width;
		re[slot]+=1;
	}
	for(int i=0;i<re.size();++i)
	{
		re[i]/=l.size();
	}
	return re;
}

void LalondeFeatures(const vector<Eigen::Vector3d>& points, vector<double>& re)
{
	rt.RemoveAll();
	for(int i=0;i<points.size();++i)
	{
		double cmax[3], cmin[3];
		for(int j=0;j<3;++j)
		{
			cmin[j]=cmax[j]=points[i](j);
		}
		rt.Insert(cmin,cmax,i);
	}
	vector<double> l1,l2,l3;
	double range=0.5;
	double num_select=20;
	for(int i=0;i<points.size();++i)
	{
		const Eigen::Vector3d& p=points[i];
		double cmax[3], cmin[3];
		for(int j=0;j<3;++j)
		{
			cmax[j]=p(j)+range;
			cmin[j]=p(j)-range;
		}
		vector<int> resultid;
		rt.Search(cmin,cmax,QueryResultCallback,resultid);
		if(resultid.size()==0)
			cout<<"NO NEIGHBOUR FROM LALONDE FEATURE!"<<endl;
		sort(resultid.begin(),resultid.end(),
			[&p,&points](const int p1id,const int p2id)
			{return dist(points[p1id],p)<dist(points[p2id],p);});
		Eigen::Vector3d center(0,0,0);
		vector<Eigen::Vector3d> nb;
		for(int j=0;j<num_select && j<resultid.size();++j)
		{
			nb.push_back(points[resultid[j]]);
			center+=points[resultid[j]];
		}
		center/=nb.size();
		Eigen::Matrix<double,3,3> cov_mat;
		for(int j=0;j<nb.size();++j)
		{
			nb[j]-=center;
			cov_mat=cov_mat+nb[j]*nb[j].transpose();
		}
		cov_mat/=nb.size();

		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(cov_mat);
		if(eigen_solver.info()==Eigen::Success)
		{
			vector<double> v;
			v.push_back(eigen_solver.eigenvalues()(0));
			v.push_back(eigen_solver.eigenvalues()(1));
			v.push_back(eigen_solver.eigenvalues()(2));
			sort(v.begin(),v.end(),
				[](const double l, const double r){return l<r;});
			double sum=v[0]+v[1]+v[2];
			l1.push_back(v[2]/sum);
			l2.push_back((v[2]-v[1])/sum);
			l3.push_back((v[1]-v[0])/sum);
		}
	}
	vector<double> l1_f=divide_bins(l1,4);
	vector<double> l2_f=divide_bins(l2,4);
	vector<double> l3_f=divide_bins(l3,4);
	for(int i=0;i<l1_f.size();++i)
		re.push_back(l1_f[i]);
	for(int i=0;i<l2_f.size();++i)
		re.push_back(l2_f[i]);
	for(int i=0;i<l3_f.size();++i)
		re.push_back(l3_f[i]);
}

void AnguelovFeatures(const vector<Eigen::Vector3d>& points, vector<double>& re)
{
	vector<double> a1,a2,a3;
	double width_range=0.1;
	double height_range=1;
	for(int i=0;i<points.size();++i)
	{
		const Eigen::Vector3d& p=points[i];
		double cmax[3], cmin[3];
		for(int j=0;j<3;++j)
		{
			double local_range;
			if(j==2)
				local_range=height_range;
			else
				local_range=width_range;
			cmin[j]=p(j)-local_range;
			cmax[j]=p(j)+local_range;
		}
		vector<int> resultid;
		rt.Search(cmin,cmax,QueryResultCallback,resultid);
		if(resultid.size()==0)
			cout<<"NO NEIGHBOUR FROM ANGUELOV FEATURE!"<<endl;
		double count1=0,count2=0,count3=0;
		for(int j=0;j<resultid.size();++j)
		{
			const Eigen::Vector3d& candi_p=points[resultid[j]];
			if(sqrt(pow(candi_p(0)-p(0),2)+pow(candi_p(1)-p(1),2))<=width_range)
			{
				double z_dist=candi_p(2)-p(2);
				if(z_dist<=height_range && z_dist>height_range/3)
					count1++;
				else if(z_dist<=height_range/3 && z_dist>-height_range/3)
					count2++;
				else if(z_dist<=-height_range/3 && z_dist>-height_range)
					count3++;
			}
		}
		double sum=count1+count2+count3;
		a1.push_back(count1/sum);
		a2.push_back(count2/sum);
		a3.push_back(count3/sum);
	}
	vector<double> a1_f=divide_bins(a1,4);
	vector<double> a2_f=divide_bins(a2,4);
	vector<double> a3_f=divide_bins(a3,4);
	for(int i=0;i<a1_f.size();++i)
		re.push_back(a1_f[i]);
	for(int i=0;i<a2_f.size();++i)
		re.push_back(a2_f[i]);
	for(int i=0;i<a3_f.size();++i)
		re.push_back(a3_f[i]);
}

string ToSVMFormat(const vector<double>& features)
{
	string re;
	re+="0 ";
	for(int i=1;i<features.size();++i)
	{
		re+=to_string(i);
		re+=":";
		re+=to_string(features[i-1]);
		re+=" ";
	}
	return re;
}

string ToSVMFormat(const vector<double>& features, int mclass)
{
	string re;
	re+=to_string(mclass);
	re+=" ";
	for(int i=1;i<=features.size();++i)
	{
		re+=to_string(i);
		re+=":";
		re+=to_string(features[i-1]);
		re+=" ";
	}
	return re;
}

void dealwithnan(vector<double>& features)
{
	for(int i=0;i<features.size();++i)
	{
		if(isnan(features[i]))
			features[i]=0;
	}
}

void normalize(vector<double>& features)
{
	double total=0;
	for(int i=0;i<features.size();++i)
	{
		total+=pow(features[i],2);
	}
	total=sqrt(total);
	for(int i=0;i<features.size();++i)
	{
		features[i]/=total;
	}
}

string ExtractFeature(const vector<Eigen::Vector3d>& points)
{
	vector<double> features;
	BasicStatisticFeatures(points,features);
	LalondeFeatures(points,features);
	AnguelovFeatures(points,features);
	dealwithnan(features);
	normalize(features);
	return ToSVMFormat(features);
}

string ExtractTrainingFeature(const vector<Eigen::Vector3d>& points, int mclass)
{
	vector<double> features;
	BasicStatisticFeatures(points,features);
	LalondeFeatures(points,features);
	AnguelovFeatures(points,features);
	dealwithnan(features);
	normalize(features);
	return ToSVMFormat(features,mclass);
}