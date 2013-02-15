/*
 * point_3d.cpp
 *
 *  Created on: 28-Aug-2012
 *      Author: renga
 */

#include "point_3d.hpp"

Point3D::Point3D(int xx, int yy, int zz) {
	x=xx;
	y=yy;
	z=zz;
	// TODO Auto-generated constructor stub

}
bool Point3D::equals(Point3D a)
{
	return a.x==x&&a.y==y&&a.z==z;
}
/*Point3D::~Point3D() {
	// TODO Auto-generated destructor stub
}*/

