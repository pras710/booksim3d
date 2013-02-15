/*
 * routing_algorithms_3d.hpp
 *
 *  Created on: 28-Aug-2012
 *      Author: renga
 */

#ifndef ROUTING_ALGORITHMS_3D_HPP_
#define ROUTING_ALGORITHMS_3D_HPP_
using namespace std;
#include "booksim.hpp"
#include <vector>
#include <sstream>
#include <assert.h>
#include "random_utils.hpp"
#include "misc_utils.hpp"
#include "Mesh_3D.hpp"
#include "point_3d.hpp"
class Point3D;
enum  FullOddEvenLowPriorityFlags{DO_NOT_INSERT, INSERT_ALL_ODDS, INSERT_ALL_EVENS, INSERT_RANDOM};
const int LOW_PRIORITY=0, HIGH_PRIORITY=1;
int getPortFor(int me, int destz);
int getBegin(bool isOddEven, int b, int e);
int getEnd(bool isOddEven, int b, int e);
void dor_mesh3d( const Router *r, const Flit *f, int in_channel,
		OutputSet *outputs, bool inject );
//extern Point3D fromPoint;
//1=ny.2=py.3=px.4=nx.0=eject
bool isAllowed(bool even, Point3D curr, int p, const Router *r);
vector<int> formYZ2DRoute(Point3D src, Point3D dest, Point3D curr, const Router *r, int flitId);
vector<int> formXY2DRouteFull(Point3D src, Point3D dest, Point3D curr, const Router *r, const Flit* f, FullOddEvenLowPriorityFlags &flag);
void odd_even_xy( const Router *r, const Flit *f, int in_channel,
		OutputSet *outputs, bool inject );
bool isPresent(vector<int> v, int t);
void nowSpruceIt(Point3D src, Point3D dest, Point3D curr, vector<int> ret, OutputSet *out, int b, int e, const Router *r);
void odd_even_3d( const Router *r, const Flit *f, int in_channel,
		OutputSet *outputs, bool inject );
void fully_odd_even_3d( const Router *r, const Flit *f, int in_channel,
		OutputSet *outputs, bool inject );
void modified_odd_even_3d( const Router *r, const Flit *f, int in_channel,
		OutputSet *outputs, bool inject );

#endif /* ROUTING_ALGORITHMS_3D_HPP_ */
