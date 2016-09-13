#include <mex.h>
#include <matrix.h>
#include "CPP_helpers.hpp"
#include "CPP_geometry.hpp"
        
static const char *acGeomery[] = { "geometryId", "positions" }; 


#define MEX_FIND(var,name,par) \
	mxArray* var = mxGetField (par, 0, name); \
	if (!var) { \
        char msg [1000]; sprintf (msg, "cannot find '%s' field.", name); \
        mexErrMsgTxt (msg); return false; \
    } 
    
bool mat2geom (const mxArray* pMexArray, ftkGeometry& out)
{
    MEX_FIND (id,        acGeomery [0], pMexArray);
    MEX_FIND (positions, acGeomery [1], pMexArray);
    
    out.geometryId = mat2base<uint32>(id);
    
    size_t m = mxGetM(positions);
	size_t n = mxGetN(positions);

    if (m != 3)
        mexErrMsgTxt("Matrix of points is probabely transposed."); 
    if (n > FTK_MAX_FIDUCIALS)
        mexErrMsgTxt("Invalid number of points."); 
        
    out.pointsCount = (uint32) n;
    
    double* ptr = mxGetPr (positions);
    for (uint32 u = 0; u < out.pointsCount; u++)
    {
        out.positions[u].x = *ptr++;
        out.positions[u].y = *ptr++;
        out.positions[u].z = *ptr++;
    }
    
    return true;
}

mxArray* geom2mat (const std::vector<ftkGeometry>& in)
{
    if (in.empty ())
        return EMPTY_ARRAY;
        
    mxArray* pMexGeometry = mxCreateStructMatrix((mwSize)in.size (),1,2, acGeomery);
    
    for (size_t v = 0; v < in.size (); v++)
    {
        const ftkGeometry& geom = in[v];
        if (geom.pointsCount == 0)
            mexErrMsgTxt("Invalid number of points");
        
        mxArray* id = base2mat<uint32> (geom.geometryId);
        mxArray* positions = mxCreateDoubleMatrix(3, geom.pointsCount, mxREAL);
        double* ptr = mxGetPr (positions);
        for (uint32 u = 0; u < geom.pointsCount; u++)
        {
            *ptr++ = (double) geom.positions[u].x;
            *ptr++ = (double) geom.positions[u].y;
            *ptr++ = (double) geom.positions[u].z;
        }

        mxSetField (pMexGeometry, v, acGeomery[0], id); 
        mxSetField (pMexGeometry, v, acGeomery[1], positions);
    }
    return pMexGeometry;
}

