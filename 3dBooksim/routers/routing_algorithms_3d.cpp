/*
 * routing_algorithms_3d.cpp
 *
 *  Created on: 28-Aug-2012
 *      Author: renga
 */
#include "routing_algorithms_3d.hpp"
Point3D fromPoint;
int getPortFor(int me, int destz)
{
	int ret=4;
	for(int i=0;i<=destz;i++)
	{
		if(me!=i)
		{
			ret++;
		}
	}
	return ret;
}
int getBegin(bool isOddEven, int b, int e)
{
	return isOddEven?b:(e-2);
}
int getEnd(bool isOddEven, int b, int e)
{
	return isOddEven?(e-3):e;
}
void dor_mesh3d( const Router *r, const Flit *f, int in_channel,
		OutputSet *outputs, bool inject )
{
	int curr=r->GetID();
	int currx = ( curr % (gK*gN) ) % gK;
	int curry = ( curr % (gK*gN) ) / gK;
	int currz = curr / (gK*gN);
	int dest=f->dest;
	int destx = ( dest % (gK*gN) ) % gK;
	int desty = ( dest % (gK*gN) ) / gK;
	int destz = dest / (gK*gN);
	int dx=destx-currx;
	int dy=desty-curry;

	int outputPort=0;//4+gLayer-1;
	outputs->Clear();
	int dz=destz-currz;
	if(dz!=0)
	{
		outputPort=getPortFor(currz, destz);
		outputPort-=r->absents;
	}
	else if(dx!=0)
	{
		outputPort=(dx>0?3:4);//getPort(currx, dx/abs(dx), curry, currz);
		if(curry==gN-1 || curry==0)
		{
			outputPort-=1;
		}
		if(currx==gK-1)
		{
			outputPort-=1;
		}
	}
	else if(dy!=0)
	{
		outputPort=(dy>0?2:1);//getPort(currx,curry+(dy/abs(dy)),currz);
		if(curry==0)
		{
			outputPort=1;
		}
	}
	int tb=0, te=0;
	tb=0;
	te=gNumVCS-1;
	outputs->AddRange(outputPort, tb, te);
}
bool validZ(int p, int absents, int currz)
{
	for(int i=0;i<currz;i++)
	{
		if(p+absents==getPortFor(currz, i))return false;
	}
	return true;
}
//1=ny.2=py.3=px.4=nx.0=eject
bool isAllowedFull(bool even, Point3D curr, int p, const Router *r)
{
	bool ret=true;
	if(p==0)return true;
	if(even)
	{
		if(curr.z-fromPoint.z>=1 && (curr.x==fromPoint.x && curr.y==fromPoint.y))
		{

			return !(
					p<(5-r->absents)
			);
		}
	}
	else
	{
		if(abs(curr.x-fromPoint.x)==1 || abs(curr.y-fromPoint.y)==1)
		{
			return validZ(p, r->absents, curr.z);
		}
	}
	return ret;
}
//1=ny.2=py.3=px.4=nx.0=eject
bool isAllowed(bool even, Point3D curr, int p, const Router *r)
{
	bool ret=true;
	if(even)
	{
		if(curr.x-fromPoint.x==1)
		{
			return !(
					(
							(p==1&&curr.y!=0)||p==2-(curr.y==0?1:0)
					)
					||
					p>=(5-r->absents)
			);
		}
	}
	else
	{
		if(abs(curr.y-fromPoint.y)==1 || abs(curr.z-fromPoint.z)==1)
		{
			return !(p==(4-(curr.y==gN-1||curr.y==0?1:0)-(curr.x==gK-1?1:0)));
		}
	}
	return ret;
}
vector<int> formYZ2DRoute(Point3D src, Point3D dest, Point3D curr, const Router *r, int flitId)
{
	vector<int> ret;
	int dy=dest.y-curr.y;
	int dz=dest.z-curr.z;
	if(dy==0)
	{
		int g=getPortFor(curr.z,(gJumpable?dest.z:curr.z+(dz/abs(dz))))-r->absents;
		ret.push_back(g);
	}
	else
	{
		if(dy>0)
		{
			if(dz==0)
			{
				ret.push_back(2-(curr.y==0?1:0));
			}
			else
			{
				if(curr.y%2==1 || curr.equals(src))
				{
					int g=getPortFor(curr.z,(gJumpable?dest.z:curr.z+(dz/abs(dz))))-r->absents;
					ret.push_back(g);
				}
				if(dest.y%2==1 || dy!=1)
				{
					ret.push_back(2-(curr.y==0?1:0));
				}
			}
		}
		else
		{
			//1=ny.2=py.3=px.4=nx.0=eject
			ret.push_back(1);
			if(curr.x<gK-1)
			{
				if(curr.x>0)
				{
					if(curr.y%2==0)
					{
						if(dz!=0)
						{
							int g=getPortFor(curr.z, (gJumpable?dest.z:curr.z+(dz/abs(dz))))-r->absents;
							ret.push_back(g);
						}
					}
				}
			}
		}
	}
	int dx=dest.x-curr.x;
	if(dx!=0)
	{
		if(dest.x%2==0)//even destination
		{
			if(dx>0)
			{
				if(dy==0 && dz==0)
				{
					if(curr.x<gK-1)ret.push_back(3-(curr.y==gN-1||curr.y==0?1:0));
				}
			}
		}
		else
		{
			if(curr.x%2==1 && dx<0)
			{
				while(!ret.empty())
				{
					ret.pop_back();
				}
			}
			if(curr.x>0)ret.push_back(4-(curr.y==gN-1||curr.y==0?1:0)-(curr.x==gK-1?1:0));
		}
	}
	return ret;
}
vector<int> formXY2DRouteFull(Point3D src, Point3D dest, Point3D curr, const Router *r, const Flit *f, FullOddEvenLowPriorityFlags &flag)
{
	vector<int> ret;
	fromPoint.x=-1;
	fromPoint=src;
	int dx=dest.x-curr.x;
	int dy=dest.y-curr.y;
	int dz=dest.z-curr.z;

	if(dy==0)
	{
		ret.push_back(dx>0?3-((curr.y==0||curr.y==gN-1)?1:0):4-(curr.y==0||curr.y==gN-1)-(curr.x==gK-1?1:0));
		return ret;
	}
	if(dx==0)
	{
		//1=n, 2=s, 3=e, 4=w
		//1=ny.2=py.3=px.4=nx.0=eject
		int out=dy>0?(2-(curr.y==0?1:0)):1;
		ret.push_back(out);
	}
	else
	{
		if(dx>0)
		{
			if(dy==0)
			{
				//1=ny.2=py.3=px.4=nx.0=eject
				int out=3-((curr.y==0||curr.y==gN-1)?1:0);
				ret.push_back(out);
			}
			else
			{
				if(curr.x%2==1 || curr.x==src.x)
				{
					//1=ny.2=py.3=px.4=nx.0=eject
					int out=dy>0?(2-(curr.y==0?1:0)):1;
					ret.push_back(out);
				}
				if(dest.x%2==1 || dx!=1)
				{
					//1=ny.2=py.3=px.4=nx.0=eject
					int out=3-((curr.y==0||curr.y==gN-1)?1:0)/*-((curr.x==gK-1)?1:0)*/;
					ret.push_back(out);
				}
			}
		}
		else
		{
			//1=ny.2=py.3=px.4=nx.0=eject
			int out=4-((curr.y==0||curr.y==gN-1)?1:0)-(curr.x==gK-1?1:0);
			ret.push_back(out);
			if(curr.x%2==0)
			{
				//1=ny.2=py.3=px.4=nx.0=eject
				out=dy>0?(2-(curr.y==0?1:0)):1;
				ret.push_back(out);
			}
		}
	}
	if(dz!=0)
	{
		if(dest.z%2==0)//even destination
		{
			if(dz>0)
			{
				if(dx==0 && dy==0)
				{
					ret.push_back(getPortFor(curr.z, dest.z)-r->absents);
				}
				flag=/*FullOddEvenLowPriorityFlags::*/INSERT_ALL_ODDS;
			}
		}
		else//odd destination -dont go to a -ve z from x and y (no problem in receiving in z)
		{
			if(curr.z%2==1 && dz<0)//if current plane is odd, it should send z first to destination
			{
				while(!ret.empty())
				{
					ret.pop_back();
				}
				flag=/*FullOddEvenLowPriorityFlags::*/INSERT_ALL_EVENS;//we dont ve anything else to do, if the right destination is not availabe..
			}
			ret.push_back(getPortFor(curr.z, dest.z)-r->absents);
		}
	}
}
void odd_even_xy( const Router *r, const Flit *f, int in_channel,
		OutputSet *outputs, bool inject )
{
	fromPoint.x=-1;
	outputs->Clear();
	int cur=r->GetID();
	Point3D curr(( cur % (gK*gN) ) % gK, ( cur % (gK*gN) ) / gK, cur / (gK*gN));
	int dst=f->dest;
	Point3D dest(( dst % (gK*gN) ) % gK, ( dst % (gK*gN) ) / gK, dst / (gK*gN));
	int srco=f->src;
	Point3D src(( srco % (gK*gN) ) % gK, ( srco % (gK*gN) ) / gK, srco / (gK*gN));
	fromPoint=src;
	int dx=dest.x-curr.x;
	int dy=dest.y-curr.y;
	int dz=dest.z-curr.z;
	int tb=0, te=0;
	tb=0;
	te=gNumVCS-1;
	int b=getBegin(true, tb, te), e=getEnd(true, tb, te);
	if(dx==0 && dy==0)
	{
		outputs->AddRange(0, b, e);
		return;
	}
	if(dy==0)
	{
		int g=dx>0?3-((curr.y==0||curr.y==gN-1)?1:0):4-(curr.y==0||curr.y==gN-1)-(curr.x==gK-1?1:0);
		outputs->AddRange(g,b,e);
		return;
	}
	if(dx==0)
	{
		//1=n, 2=s, 3=e, 4=w
		//1=ny.2=py.3=px.4=nx.0=eject
		int out=dy>0?(2-(curr.y==0?1:0)):1;
		outputs->AddRange(out, b, e);
	}
	else
	{
		if(dx>0)
		{
			if(dy==0)
			{
				//1=ny.2=py.3=px.4=nx.0=eject
				int out=3-((curr.y==0||curr.y==gN-1)?1:0);
				outputs->AddRange(out, b, e);
			}
			else
			{
				if(curr.x%2==1 || curr.x==src.x)
				{
					//1=ny.2=py.3=px.4=nx.0=eject
					int out=dy>0?(2-(curr.y==0?1:0)):1;
					outputs->AddRange(out, b, e);
				}
				if(dest.x%2==1 || dx!=1)
				{
					//1=ny.2=py.3=px.4=nx.0=eject
					int out=3-((curr.y==0||curr.y==gN-1)?1:0)/*-((curr.x==gK-1)?1:0)*/;
					outputs->AddRange(out, b, e);
				}
			}
		}
		else
		{
			//1=ny.2=py.3=px.4=nx.0=eject
			int out=4-((curr.y==0||curr.y==gN-1)?1:0)-(curr.x==gK-1?1:0);
			outputs->AddRange(out, b, e);
			if(curr.x%2==0)
			{
				//1=ny.2=py.3=px.4=nx.0=eject
				out=dy>0?(2-(curr.y==0?1:0)):1;
				outputs->AddRange(out, b, e);
			}
		}
	}
}

bool isPresent(vector<int> v, int t)
{
	for(int i=0;i<v.size();i++)
	{
		if(v[i]==t)return true;
	}
	return false;
}
void nowSpruceIt(Point3D src, Point3D dest, Point3D curr, vector<int> ret, OutputSet *out, int b, int e, const Router *r)
{
	vector<int> tempCopy;

	if(fromPoint.x!=-1)
	{
		for(vector<int>::iterator it=ret.begin();it!=ret.end();it++)
		{
			int p=*it;
			if(isAllowed(curr.x%2==0, curr, p, r))
			{
				if(!isPresent(tempCopy, p))
				{
					tempCopy.push_back(p);
					out->AddRange(p,b,e);
				}
			}
		}
	}
	else
	{
		for(vector<int>::iterator it=ret.begin();it!=ret.end();it++)
		{
			if(!isPresent(tempCopy, *it))
			{
				tempCopy.push_back(*it);
				out->AddRange(*it,b,e);
			}
		}
	}
}
void odd_even_3d( const Router *r, const Flit *f, int in_channel,
		OutputSet *outputs, bool inject )
{
	fromPoint.x=-1;
	outputs->Clear();
	int cur=r->GetID();
	Point3D curr(( cur % (gK*gN) ) % gK, ( cur % (gK*gN) ) / gK, cur / (gK*gN));
	int dst=f->dest;
	Point3D dest(( dst % (gK*gN) ) % gK, ( dst % (gK*gN) ) / gK, dst / (gK*gN));
	int srco=f->src;
	Point3D src(( srco % (gK*gN) ) % gK, ( srco % (gK*gN) ) / gK, srco / (gK*gN));
	if(f->myPreviousVisit==-1)
	{
		fromPoint.x=-1;
	}
	else
	{
		fromPoint.x=(f->myPreviousVisit%(gK*gN))%gK;
		fromPoint.y=(f->myPreviousVisit%(gK*gN))/gK;
		fromPoint.z=(f->myPreviousVisit/(gK*gN));
	}
	int dx=dest.x-curr.x;
	int dy=dest.y-curr.y;
	int dz=dest.z-curr.z;
	int tb=0,te=0;
	tb=0;
	te=gNumVCS-1;
	int b=tb, e=te;
	if(dx==0 && dy==0 && dz==0)
	{
		outputs->AddRange(/*4+gLayer-1*/0,b,e);
		return;
	}
	if(dx==0 && dy==0)
	{
		int g=getPortFor(curr.z, (gJumpable?dest.z:curr.z+(dz/abs(dz))))-r->absents;
		outputs->AddRange(g,b,e);
		return;
	}
	if(dx==0 && dz==0)
	{
		outputs->AddRange((dy>0?2-(curr.y==0?1:0):1),b,e);
		return;
	}
	if(dz==0 && dy==0)
	{
		outputs->AddRange(dx>0?3-((curr.y==0||curr.y==gN-1)?1:0):4-(curr.y==0||curr.y==gN-1)-(curr.x==gK-1?1:0),b,e);
		return;
	}
	nowSpruceIt(src, dest, curr, formYZ2DRoute(src, dest, curr, r, f->id), outputs, b, e, r);
	fromPoint.x=-1;
}
void nowSpruceItFully(Point3D src, Point3D dest, Point3D curr, vector<int> ret, OutputSet *out, int b, int e, const Router *r, const FullOddEvenLowPriorityFlags flag)
{
	vector<int> tempCopy;

	if(fromPoint.x!=-1)
	{
		for(vector<int>::iterator it=ret.begin();it!=ret.end();it++)
		{
			int p=*it;
			if(isAllowedFull(curr.z%2==0, curr, p, r)||ret.size()==1)
			{
				if(!isPresent(tempCopy, p))
				{
					tempCopy.push_back(p);
					out->AddRange(p,b,e, HIGH_PRIORITY);
				}
			}
		}
	}
	else
	{
		for(vector<int>::iterator it=ret.begin();it!=ret.end();it++)
		{
			if(!isPresent(tempCopy, *it))
			{
				tempCopy.push_back(*it);
				out->AddRange(*it,b,e, HIGH_PRIORITY);
			}
		}
	}
	for(int i=0;i<gLayer;i++)
	{
		if(i==curr.z)continue;
		if((i%2==1 && /*FullOddEvenLowPriorityFlags::*/INSERT_ALL_ODDS==flag)||(i%2==0 && /*FullOddEvenLowPriorityFlags::*/INSERT_ALL_EVENS==flag))
		{
			out->AddRange(getPortFor(curr.z, i)-r->absents, b, e, LOW_PRIORITY);
		}
	}
	if(ret.size()==0)
	{
		cout<<"xy routing is screwed..";
		exit(-1);
	}
	if(tempCopy.size()==0 && ret.size()>0)
	{

		cout<<"Sprucing is screwed!!!\ncurr="<<curr.x<<" "<<curr.y<<" "<<curr.z<<"\ndest: "<<dest.x<<" "<<dest.y<<" "<<dest.z<<endl;;
		exit(-1);
	}
}
void fully_odd_even_3d( const Router *r, const Flit *f, int in_channel,
		OutputSet *outputs, bool inject )
{
	outputs->Clear();
	int cur=r->GetID();
	Point3D curr(( cur % (gK*gN) ) % gK, ( cur % (gK*gN) ) / gK, cur / (gK*gN));
	int dst=f->dest;
	Point3D dest(( dst % (gK*gN) ) % gK, ( dst % (gK*gN) ) / gK, dst / (gK*gN));
	int srco=f->src;
	Point3D src(( srco % (gK*gN) ) % gK, ( srco % (gK*gN) ) / gK, srco / (gK*gN));
	int dx=dest.x-curr.x;
	int dy=dest.y-curr.y;
	int dz=dest.z-curr.z;
	int tb=0,te=0;
	tb=0;
	te=gNumVCS-1;
	int b=tb/*;getBegin(true, tb, te)*/, e=te;/*getEnd(true, tb, te);*/
	if(dx==0 && dy==0 && dz==0)
	{
		outputs->AddRange(/*4+gLayer-1*/0,b,e);
		return;
	}
	if(dx==0 && dy==0)
	{
		int g=getPortFor(curr.z, dest.z)-r->absents;
		outputs->AddRange(g,b,e);
		return;
	}
	if(dx==0 && dz==0)
	{
		outputs->AddRange((dy>0?2-(curr.y==0?1:0):1),b,e);
		return;
	}
	if(dz==0 && dy==0)
	{
		outputs->AddRange(dx>0?3-((curr.y==0||curr.y==gN-1)?1:0):4-(curr.y==0||curr.y==gN-1)-(curr.x==gK-1?1:0),b,e);
		return;
	}
	if(f->myPreviousVisit==-1)
	{
		fromPoint.x=-1;
	}
	else
	{
		fromPoint.x=(f->myPreviousVisit%(gK*gN))%gK;
		fromPoint.y=(f->myPreviousVisit%(gK*gN))/gK;
		fromPoint.z=(f->myPreviousVisit/(gK*gN));
	}
	FullOddEvenLowPriorityFlags flag=/*FullOddEvenLowPriorityFlags::*/DO_NOT_INSERT;
	nowSpruceItFully(src, dest, curr, formXY2DRouteFull(src, dest, curr, r, f, flag), outputs, b, e, r, flag);
	fromPoint.x=-1;
}
void modified_odd_even_3d( const Router *r, const Flit *f, int in_channel,
		OutputSet *outputs, bool inject )
{
	outputs->Clear();
	int dz=f->dest/(gK*gN)-r->GetID()/(gK*gN);
	if(dz==0)
	{
		odd_even_xy(r, f, in_channel, outputs, inject);
	}
	else
	{
		int g=getPortFor(r->GetID()/(gK*gN),f->dest/(gK*gN))-r->absents;
		outputs->AddRange(g, getBegin(false, 0, gNumVCS-1), getEnd(false, 0, gNumVCS-1));
	}
}
