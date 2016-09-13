#ifndef CPP_geometry_hpp
#define CPP_geometry_hpp

	#include <ftkInterface.h>
        
    bool     mat2geom (const mxArray* pMexArray, ftkGeometry& out);
    mxArray* geom2mat (const std::vector<ftkGeometry>& in);
        
#endif