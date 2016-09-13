#include <matrix.h>
#include "CPP_helpers.hpp"
#include "CPP_fusionTrack.hpp"
#include "CPP_geometry.hpp"
#include "CPP_frame.hpp"
#include "CPP_option.hpp"

void mexFunction(int nOut, mxArray *pOut[], int nIn, const mxArray *pIn[])
{	
    // Get the command string
    char cmd[64];
	if (nIn < 1 || mxGetString(pIn[0], cmd, sizeof(cmd)))
		mexErrMsgTxt("First input should be a command string less than 64 characters long.");
        
    if (nOut < 0)
        mexErrMsgTxt("DEVICES: Invalid output arguments.");

    // --------------- New ---------------------
    
    if (!strcmp("new", cmd)) 
    {
        // Check parameters
        if (nOut != 1)
            mexErrMsgTxt("NEW: One output expected.");
        // Return a handle to a new C++ instance
        pOut[0] = ptr2mat<fusionTrack>(new fusionTrack);
        return;
    }
    
    // Check there is a second input, which should be the class instance handle
    if (nIn < 2)
		mexErrMsgTxt("Second input should be a class instance handle.");
    
    // --------------- Delete ---------------------
    
    if (!strcmp("delete", cmd)) 
    {
        destroy<fusionTrack>(pIn[1]);
        // Warn if other commands were ignored
        if (nOut != 0 || nIn != 2)
            mexWarnMsgTxt("DELETE: Unexpected arguments ignored.");
        return;
    }
	
	// Get the class instance pointer from the second input
    fusionTrack *pFusionTrack = mat2ptr<fusionTrack>(pIn[1]);
    
    // --------------- METHODS ---------------------
    
    if (!strcmp("devices", cmd)) 
    {
        std::vector<uint64> devices = pFusionTrack->devices();
        pOut[0] = vec2mat<uint64>(devices);
        return;
    }
    
    if (!strcmp("geometries", cmd)) 
    {
        if (nIn < 3)
            mexErrMsgTxt("Invalid number of input arguments.");
        uint64 device = mat2base<uint64>(pIn[2]);
        std::vector<ftkGeometry> geometries = pFusionTrack->geometries (device);
        pOut[0] = geom2mat (geometries);
        return;
    }
    
    if (!strcmp("options", cmd)) 
    {
        if (nIn < 3)
            mexErrMsgTxt("Invalid number of input arguments.");
        uint64 device = mat2base<uint64>(pIn[2]);
        std::vector<ftkOptionsInfo> options = pFusionTrack->options (device);
        pOut[0] = opt2mat (options);
        return;
    }
    
    if (!strcmp("cleargeometry", cmd)) 
    {
        if (nIn < 4)
            mexErrMsgTxt("Invalid number of input arguments.");
        uint64 device = mat2base<uint64>(pIn[2]);
        uint32 geomId = mat2base<uint32>(pIn[3]);     
        pFusionTrack->clearGeometry (device, geomId);
        return;
    }
    
    if (!strcmp("setgeometry", cmd)) 
    {
       if (nIn < 4)
            mexErrMsgTxt("Invalid number of input arguments.");
        uint64 device = mat2base<uint64>(pIn[2]);
        ftkGeometry geom;
        if (!mat2geom (pIn[3], geom))
            mexErrMsgTxt("Invalid geometry.");
        pFusionTrack->setGeometry (device, geom);
        return;
    }
    
    if (!strcmp("getint32", cmd)) 
    {
        if (nIn < 4)
            mexErrMsgTxt("Invalid number of input arguments.");
        uint64 device = mat2base<uint64>(pIn[2]);
        uint32 optId  = mat2base<uint32>(pIn[3]);
        ftkOptionGetter what = nIn < 5 ? FTK_VALUE : (ftkOptionGetter) mat2base<uint8> (pIn[4]);
        pOut[0] = base2mat<int32>(pFusionTrack->getInt32(device, optId, what));
        return;
    }
    
    if (!strcmp("setint32", cmd)) 
    {
        if (nIn < 5)
            mexErrMsgTxt("Invalid number of input arguments.");
        uint64 device = mat2base<uint64>(pIn[2]);
        uint32 optId  = mat2base<uint32>(pIn[3]);        
        int32  val    = mat2base<int32> (pIn[4]);        
        pFusionTrack->setInt32(device, optId, val);
        return;
    }
    
     if (!strcmp("getfloat32", cmd)) 
    {
        if (nIn < 4)
            mexErrMsgTxt("Invalid number of input arguments.");
        uint64 device = mat2base<uint64>(pIn[2]);
        uint32 optId  = mat2base<uint32>(pIn[3]);
        ftkOptionGetter what = nIn < 5 ? FTK_VALUE : (ftkOptionGetter) mat2base<uint8> (pIn[4]);
        pOut[0] = base2mat<float32>(pFusionTrack->getFloat32(device, optId, what));
        return;
    }
    
    if (!strcmp("setfloat32", cmd)) 
    {
        if (nIn < 5)
            mexErrMsgTxt("Invalid number of input arguments.");
        uint64  device = mat2base<uint64>(pIn[2]);
        uint32  optId  = mat2base<uint32>(pIn[3]);        
        float32 val    = mat2base<float32> (pIn[4]);        
        pFusionTrack->setFloat32(device, optId, val);
        return;
    }
    
    if (!strcmp("getframe", cmd)) 
    {
        if (nIn < 3)
            mexErrMsgTxt("Invalid number of input arguments.");
        uint64 device = mat2base<uint64>(pIn[2]);
        pFusionTrack->getLastFrame(device);
        pOut[0] = frame2mat(pFusionTrack->getFrame ());
        return;
    }

}
