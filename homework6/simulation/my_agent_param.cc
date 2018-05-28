#include <iostream>
#include <cmath>
#include "my_agent_param.h"
using namespace std;

//double my_agent_pid[3]={10,2,0};
//double my_agent_pid[3]={14.9085, 0.574384, -0.0480859};
//double my_agent_pid[3]={9.93098, 2.17319, -0.000706965};
//double my_agent_pid[3]={10.1452, 2.07415, -0.000967527};
double my_agent_pid[3]={-2, 0, 0};

double err=0;
double best_err=0;
bool inited=false;

double dpid[3]={1,0.1,0.1};

double tol=0.001;

int curri=0;

bool isbeforefirstif=true;
bool isinfirstif=false;
bool isbeforesecondif=false;

void updateparam()
{
  if(!inited)
  {
    best_err=err;
    inited=true;
    return;
  }
  double currtol=dpid[0]+dpid[1]+dpid[2];
  cout<<"Currnt Tol: "<<currtol<<endl;
  if(currtol<tol)
  {
    cout<<"Done!"<<endl;
    cout<<"P: "<<my_agent_pid[0]<<endl;
    cout<<"I: "<<my_agent_pid[1]<<endl;
    cout<<"D: "<<my_agent_pid[2]<<endl;
    int temp;
    cin>>temp;
  }
  cout<<"Current Error: "<<err<<endl;
  cout<<"Best Error: "<<best_err<<endl;
  cout<<"Current PID: "<<my_agent_pid[0]<<", "<<my_agent_pid[1]<<", "<<my_agent_pid[2]<<endl<<endl;
  if(isbeforefirstif)
  {
    my_agent_pid[curri]+=dpid[curri];
    isbeforefirstif=false;
    isinfirstif=true;
    return;
  }
  if(isinfirstif)
  {
    if(abs(err)<abs(best_err))
    {
      best_err=err;
      dpid[curri]*=1.1;
      curri=(curri+1)%3;
      isbeforefirstif=true;
      isinfirstif=false;
      return;
    }
    else
    {
      my_agent_pid[curri]-=2*dpid[curri];
      isbeforesecondif=true;
      isinfirstif=false;
      return;
    }
  }
  if(isbeforesecondif)
  {
    if(abs(err)<abs(best_err))
    {
      best_err=err;
      dpid[curri]*=1.1;
      curri=(curri+1)%3;
      isbeforesecondif=false;
      isbeforefirstif=true;
      return;
    }
    else
    {
      my_agent_pid[curri]+=dpid[curri];
      dpid[curri]*=0.9;
      curri=(curri+1)%3;
      isbeforesecondif=false;
      isbeforefirstif=true;
      return;
    }
  }
}