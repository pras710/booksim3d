/*
 * point_3d.hpp
 *
 *  Created on: 28-Aug-2012
 *      Author: renga
 */

#ifndef POINT_3D_HPP_
#define POINT_3D_HPP_

class Point3D
{
public:
	int x,y,z;
	Point3D(){}
	//ostream operator <<(ostream c){c<<x<<" "<<y<<" "<<z;return c;}
	Point3D(int xx,int yy, int zz);//{x=xx;y=yy;z=zz;}
	bool equals(Point3D a);//{return a.x==x&&a.y==y&&a.z==z;}
};

#endif /* POINT_3D_HPP_ */
