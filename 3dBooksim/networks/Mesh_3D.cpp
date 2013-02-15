/*
 * Mesh3D.cpp
 *
 *  Created on: 17-Jul-2012
 *      Author: renga
 */
#include "booksim.hpp"
#include <vector>
#include <sstream>
#include <assert.h>
#include "random_utils.hpp"
#include "misc_utils.hpp"
#include "Mesh_3D.hpp"
#include "routing_algorithms_3d.hpp"
int Mesh3D::_cX = 0 ;
int Mesh3D::_cY = 0 ;
int Mesh3D::_cZ = 0 ;
Mesh3D::Mesh3D( const Configuration &config, const string & name ): Network(config, name)
{
	//compute size, alloc and build net are the three functions...
	_ComputeSize(config);
	_Alloc();
	_BuildNet(config);
}
void Mesh3D::_ComputeSize(const Configuration &config)
{
	int k = config.GetInt( "k" );//here this is number of nodes in x direction
	int n = config.GetInt( "n" );//this is number of nodes in y direction
	int c = config.GetInt( "c" );
	int layer = config.GetInt( "layer" );//this is number of nodes in z direction
	//how many routers in the x or y or z direction
	xcount = config.GetInt("x");
	ycount = config.GetInt("y");
	zcount = config.GetInt("z");
	//configuration of hohw many clients in X and Y per router
	xrouter = config.GetInt("xr");
	yrouter = config.GetInt("yr");
	_express_channels = (config.GetInt("express_channels") == 1);

	gK = _k = k ;
	gN = _n = n ;
	gC = _c = c ;
	gLayer = _layer = layer;
	assert(c == xrouter*yrouter);

	_sources  = k*n*layer;//_c * powi( _k, _n) * layer; // Source nodes in network
	_dests    = k*n*layer;//_c * powi( _k, _n) * layer; // Destination nodes in network
	_size     = k*n*layer;//powi( _k, _n);      // Number of routers in network
	_channels = (4+_layer-1) * _size;// * _size + _size*(layer-1);   //top+bottom+north east west south  // Number of channels in network
	_cX = _c / _n ;   // Concentration in X Dimension
	_cX=_cX==0?1:_cX;
	_cY = _c / _cX ;  // Concentration in Y Dimension
	_cY=_cY==0?1:_cY;
	//
	/*_memo_NodeShiftX = _cX >> 1 ;
	_memo_NodeShiftY = log_two(gK * _cX) + ( _cY >> 1 ) ;
	_memo_PortShiftY = log_two(gK * _cX)  ;*/
}
int getInCount(int me, int in, int layers)
{
	int ret=0;
	for(int i=0;i<layers;i++)
	{
		if(in==i)
		{
			return ret;
		}
		if(me!=i)
		{
			ret++;
		}
	}
	return ret;
}
bool inRange(int n, int end)
{
	return n>=0 && n<end;
}
void Mesh3D::_BuildNet( const Configuration& config )
{
	int x_index ;
	int y_index ;
	int z_index;
	//standard trace configuration
	if(gTrace){
		cout<<"Mesh 3d in build net, Setup Finished Router"<<endl;
	}

	//latency type, noc or conventional network
	bool use_noc_latency;
	use_noc_latency = (config.GetInt("use_noc_latency")==1);

	ostringstream name;
	// The following vector is used to check that every
	//  processor in the system is connected to the network
	bool* channel_vector = new bool [ _sources ] ;
	for ( int i = 0 ; i < _sources ; i++ )
		channel_vector[i] = false ;

	//
	// Routers and Channel
	//
	for (int node = 0; node < _size; ++node) {

		// Router index derived from mesh index
		z_index = node / (_k*_n);
		y_index = ( node % (_k*_n) ) / _k;
		x_index = ( node % (_k*_n) ) % _k;
		int subs=0;
		if(x_index==0)
		{
			subs++;
		}
		if(y_index==0)
		{
			subs++;
		}
		if(x_index==_k-1)
		{
			subs++;
		}
		if(y_index==_n-1)
		{
			subs++;
		}
		const int degree_in  = (4-subs+_layer);//2 *_n + _c ;
		const int degree_out = (4-subs+_layer);//2 *_n + _c ;

		name << "router_" <<z_index<<"_"<< y_index <<"_"<< x_index;
		_routers[node] = Router::NewRouter( config,
				this,
				name.str(),
				node,
				degree_in,
				degree_out);
		_routers[node]->absents=subs;
		name.str("");

		//
		// Port Numbering: as best as I can determine, the order in
		//  which the input and output channels are added to the
		//  router determines the associated port number that must be
		//  used by the router. Output port number increases with
		//  each new channel
		//

		//
		// Processing node channels
		//
		//cout<<"cx="<<_cX<<" cy="<<_cY<<" k="<<_k<<endl;
		//int link = (_k * _cX) * (_cY * y_index + y) + (_cX * x_index + x) ;
		//OLD:k*y_index+x
		//NEW:k*n*z_index+k*y_index+x;
		int link = (_k*_n)*(z_index)+(_k ) * ( y_index) + (x_index ) ;
		assert( link >= 0 ) ;
		assert( link < _sources ) ;
		assert( link < _dests ) ;
		assert( channel_vector[ link ] == false ) ;
		channel_vector[link] = true ;
		// Ingress Ports
		_routers[node]->AddInputChannel(_inject[link], _inject_cred[link]);
		// Egress Ports
		_routers[node]->AddOutputChannel(_eject[link], _eject_cred[link]);
		//injeciton ejection latency is 1
		_inject[link]->SetLatency( 1 );
		_eject[link]->SetLatency( 1 );

		//
		// router to router channels
		//
		const int x = ( node % (_k*_n) ) % _k;
		const int y = ( node % (_k*_n) ) / _k;
		const int z = node / (_k*_n);

		const int offset = (_k*_n*_layer);//_k*_n*_layer;/*powi( _k, _n ) */;
		//the channel number of the input output channels.
		//node = (z * (_k*_n)) + y * _k + x;
		int latency=1;
		//(z) * (_k*_n)) + y * _k + x
		int px_in  = (z*_k*_n)+_k * y + ((x+1)) + 1 * offset ;
		int nx_in  = (z*_k*_n)+_k * y + ((x-1)) + 0 * offset ;
		int py_in  = (z*_k*_n)+_k * ((y+1)) + x + 3 * offset ;
		int ny_in  = -1;
		if((z*_k*_n)+_k * ((y-1)) + x >=0)
		{
			ny_in = (z*_k*_n)+_k * ((y-1)) + x + 2 * offset ;
		}
		if(y!=0/*inRange(ny_in,_chan.size())*/)
		{
			_routers[node]->AddInputChannel(_chan[ny_in],_chan_cred[ny_in]);
		}
		if(y!=_n-1)
		{
			_routers[node]->AddInputChannel(_chan[py_in],_chan_cred[py_in]);
		}
		if(x!=_k-1)
		{
			_routers[node]->AddInputChannel(_chan[px_in],_chan_cred[px_in]);
		}
		if(x!=0)
		{
			_routers[node]->AddInputChannel(_chan[nx_in],_chan_cred[nx_in]);
		}
		if(y!=0)
		{
			int ny_out = node + 3 * offset ;
			_chan[ny_out]->SetLatency(latency);
			_chan_cred[ny_out]->SetLatency(latency);
			_routers[node]->AddOutputChannel(_chan[ny_out],_chan_cred[ny_out]);
		}
		if(y!=_n-1)
		{
			int py_out = node + 2 * offset ;
			_chan[py_out]->SetLatency(latency);
			_chan_cred[py_out]->SetLatency(latency);
			_routers[node]->AddOutputChannel(_chan[py_out],_chan_cred[py_out]);
		}
		if(x!=_k-1)
		{
			int px_out = node + 0 * offset ;
			_chan[px_out]->SetLatency(latency);
			_chan_cred[px_out]->SetLatency(latency);
			_routers[node]->AddOutputChannel(_chan[px_out],_chan_cred[px_out]);
		}
		if(x!=0)
		{
			int nx_out = node + 1 * offset ;
			_chan[nx_out]->SetLatency(latency);
			_chan_cred[nx_out]->SetLatency(latency);
			_routers[node]->AddOutputChannel(_chan[nx_out],_chan_cred[nx_out]);
		}
		int count=0;
		for(int lay=0;lay<_layer;lay++)
		{
			if(lay==z)continue;
			int pz_out = ((z) * (_k*_n)) + y * _k + x + ((4+count) * offset) ;
			int pz_in=((lay) * (_k*_n)) + y * _k + x + ((4+(lay>z?z:z-1)) * offset) ;//getInCount(/*z+*/debug_z, lay, _layer);
			count++;
			_chan[pz_out]->SetLatency(1);
			_chan_cred[pz_out]->SetLatency(1);
			_chan[pz_out]->isZChannel=true;
			_routers[node]->AddInputChannel(_chan[pz_in], _chan_cred[pz_in]);
			_routers[node]->AddOutputChannel(_chan[pz_out], _chan_cred[pz_out]);
		}
		/*set latency and add the channels*/
	}
	// Check that all processors were connected to the network
	for ( int i = 0 ; i < _sources ; i++ )
	{
		assert( channel_vector[i] == true ) ;
	}

	if(gTrace){
		cout<<"Setup Finished Link"<<endl;
	}
}
int Mesh3D::GetN() const
{
	return _n;
}
int Mesh3D::GetK() const
{
	return _k;
}
int Mesh3D::GetLayer() const
{
	return _layer;
}
int Mesh3D::NodeToRouter( int address )
{
	return -1;
}
int Mesh3D::NodeToPort( int address )
{
	return -1;
}
void Mesh3D::RegisterRoutingFunctions()
{
	gRoutingFunctionMap["dor3d_mesh3d"] = &dor_mesh3d;//dimension order
	gRoutingFunctionMap["odd_even3d_mesh3d"]=&odd_even_3d;//ordinary odd even with yz following 2d odd even and x-yz following extended odd even algorithm as mentioned in Chiu [odd even turn model for adaptive routing, 2000, IPDS]
	gRoutingFunctionMap["fully_odd_even3d_mesh3d"]=&fully_odd_even_3d; //this is to demonstrate the same algorithm in a 3d noc. odd even can do jumps along z plane.
	gRoutingFunctionMap["modified_odd_even3d_mesh3d"]=&modified_odd_even_3d;//this does a z jump in the hop, then does odd even along xy.
}
