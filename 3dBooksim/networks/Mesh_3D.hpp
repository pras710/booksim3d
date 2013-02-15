/*
 * Mesh_3D.hpp
 *
 *  Created on: 17-Jul-2012
 *      Author: renga
 */

#ifndef MESH_3D_HPP_
#define MESH_3D_HPP_

#include "network.hpp"
#include "routefunc.hpp"

class Mesh3D : public Network {
public:
  Mesh3D( const Configuration &config, const string & name );
  int GetN() const;
  int GetK() const;
  int GetLayer() const;
  static int NodeToRouter( int address ) ;
  static int NodeToPort( int address ) ;

  static void RegisterRoutingFunctions() ;

private:

  static int _cX ;
  static int _cY ;
  static int _cZ;
  /*static int _memo_NodeShiftX ;
  static int _memo_NodeShiftY ;
  static int _memo_PortShiftY ;*/

  void _ComputeSize( const Configuration &config );
  void _BuildNet( const Configuration& config );

  int _k ;
  int _n ;
  int _layer;
  int _c ;
  bool _express_channels;
};

//
// Routing Functions
//
/*void dor_mesh3d( const Router *r, const Flit *f, int in_channel,
		  OutputSet *outputs, bool inject ) ;
void odd_even_3d( const Router *r, const Flit *f, int in_channel,
		  OutputSet *outputs, bool inject ) ;*/
/*
void xy_yx_cmesh( const Router *r, const Flit *f, int in_channel,
		  OutputSet *outputs, bool inject ) ;

void xy_yx_no_express_cmesh( const Router *r, const Flit *f, int in_channel,
			     OutputSet *outputs, bool inject ) ;

void dor_cmesh( const Router *r, const Flit *f, int in_channel,
		OutputSet *outputs, bool inject ) ;

void dor_no_express_cmesh( const Router *r, const Flit *f, int in_channel,
			   OutputSet *outputs, bool inject ) ;
*/



#endif /* MESH_3D_HPP_ */
