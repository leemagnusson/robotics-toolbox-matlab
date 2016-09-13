#include <mex.h>
#include <matrix.h>
#include "CPP_helpers.hpp"
#include "CPP_option.hpp"
        
static const char *acOption[]  = 
    {"id", "component", "statusRead", "statusWrite", 
     "type", "name", "description", "unit" }; 

mxArray* opt2mat (const std::vector<ftkOptionsInfo>& in)
{
    if (in.empty ())
        return EMPTY_ARRAY;
    
    mxArray* pMexOptions = mxCreateStructMatrix((mwSize)in.size (),1,8, acOption);
    
    for (uint32 u = 0; u < in.size (); u++)
    {
        const ftkOptionsInfo& option = in[u];
 
        mxArray* pId  = base2mat<uint32> (option.id);
        mxSetField (pMexOptions, (mwIndex) u, acOption[0], pId); 

        mxArray* pComponent;
        switch (option.component)
        {
            case FTK_LIBRARY:   pComponent = mxCreateString ("LIBRARY"); break;
            case FTK_DEVICE:    pComponent = mxCreateString ("DEVICE"); break;
            case FTK_DETECTOR:  pComponent = mxCreateString ("DETECTOR"); break;
            case FTK_MATCH2D3D: pComponent = mxCreateString ("MATCH2D3D"); break;
            default:            pComponent = mxCreateString ("???"); break;
        }
        mxSetField (pMexOptions, (mwIndex) u, acOption[1], pComponent); 

        mxArray* pStatusRead  = base2mat<uint8> (option.status.read > 0 ? 1:0);
        mxSetField (pMexOptions, (mwIndex) u, acOption[2], pStatusRead); 
        mxArray* pStatusWrite  = base2mat<uint8> (option.status.write > 0 ? 1:0);
        mxSetField (pMexOptions, (mwIndex) u, acOption[3], pStatusWrite); 
        
        mxArray* pType;
        switch (option.type)
        {
            case FTK_INT32:   pType = mxCreateString ("INT32"); break;
            case FTK_FLOAT32: pType = mxCreateString ("FLOAT32"); break;
            case FTK_DATA:    pType = mxCreateString ("DATA"); break;
            default:          pType = mxCreateString ("???"); break;
        }
        mxSetField (pMexOptions, (mwIndex) u, acOption[4], pType);         
        
        mxArray* pName = mxCreateString (option.name);
        mxSetField (pMexOptions, (mwIndex) u, acOption[5], pName); 

        mxArray* pDescription = mxCreateString (option.description);
        mxSetField (pMexOptions, (mwIndex) u, acOption[6], pDescription); 

        mxArray* pUnit = mxCreateString (option.unit);
        mxSetField (pMexOptions, (mwIndex) u, acOption[7], pUnit);
    }
    return pMexOptions;
}

