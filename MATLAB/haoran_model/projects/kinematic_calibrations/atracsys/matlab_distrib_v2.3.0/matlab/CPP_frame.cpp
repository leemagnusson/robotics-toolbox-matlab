#include <mex.h>
#include <matrix.h>
#include "CPP_helpers.hpp"
#include "CPP_frame.hpp"
        
static const char *acFrame[]  = { "threeDFiducials", "markers" }; 
static const char *acTDF[]    = { "positionMM", "epipolarErrorPixels", "triangulationErrorMM" }; 
static const char *acMarker[] = { "trackingId", "geometryId", "geometryPresenceMask", "fiducialCorresp", 
                                  "rotation", "translationMM", "registrationErrorMM"}; 


static inline mxArray* mrk2mat (const ftkMarker* in, const uint32 count)
{
    if (in == NULL || !count)
        return EMPTY_ARRAY;
    
    mxArray* pMexMRK = mxCreateStructMatrix(count,1,7, acMarker);
    
    for (uint32 u = 0; u < count; u++)
    {
        const ftkMarker& marker = in[u];

        mxArray* pId  = base2mat<uint32> (marker.id);
        mxSetField (pMexMRK, (mwIndex) u, acMarker[0], pId); 

        mxArray* pGeometryId  = base2mat<uint32> (marker.geometryId);
        mxSetField (pMexMRK, (mwIndex) u, acMarker[1], pGeometryId); 

        mxArray* pGeometryPresMask  = base2mat<uint32> (marker.geometryPresenceMask);
        mxSetField (pMexMRK, (mwIndex) u, acMarker[2], pGeometryPresMask); 

        mxArray* pFiducialCorresp  = mxCreateDoubleMatrix(FTK_MAX_FIDUCIALS, 1, mxREAL);
        double* ptr = mxGetPr (pFiducialCorresp);
        for (uint32 v = 0; v < FTK_MAX_FIDUCIALS; v++)
            *ptr++ = marker.fiducialCorresp [v] == INVALID_ID ? (double) -1.0: (double) (marker.fiducialCorresp [v] + 1);
        mxSetField (pMexMRK, (mwIndex) u, acMarker[3], pFiducialCorresp); 

        mxArray* pRotation = mxCreateDoubleMatrix(3, 3, mxREAL);
        ptr = mxGetPr (pRotation);
        for (uint32 v = 0; v < 3; v++)
            for (uint32 w = 0; w < 3; w++)
                *ptr++ = (double) marker.rotation [w][v];
        mxSetField (pMexMRK, (mwIndex) u, acMarker[4], pRotation); 

        mxArray* pTranslationMM  = mxCreateDoubleMatrix(3, 1, mxREAL);
        ptr = mxGetPr (pTranslationMM);
        for (uint32 v = 0; v < 3; v++)
            *ptr++ = (double) marker.translationMM [v];
        mxSetField (pMexMRK, (mwIndex) u, acMarker[5], pTranslationMM); 
        
        mxArray* pRegistrationErrorMM  = base2mat<float32> (marker.registrationErrorMM);
        mxSetField (pMexMRK, (mwIndex) u, acMarker[6], pRegistrationErrorMM); 
    }
    return pMexMRK;
}

static inline mxArray* tdf2mat (const ftk3DFiducial* in, const uint32 count)
{
    if (in == NULL || !count)
        return EMPTY_ARRAY;
    
    mxArray* pMexTDF = mxCreateStructMatrix(count,1,3, acTDF);
    
    for (uint32 u = 0; u < count; u++)
    {
        const ftk3DFiducial& fid = in[u];
        mxArray* pEEP  = base2mat<float32> (fid.epipolarErrorPixels);
        mxArray* pTEMM = base2mat<float32> (fid.triangulationErrorMM);
        mxArray* pPOS  = mxCreateDoubleMatrix(3, 1, mxREAL);
        double* ptr = mxGetPr (pPOS);
        
        // mexPrintf("%lf\t%lf\t%lf\n", fid.positionMM.x, fid.positionMM.y, fid.positionMM.z); 
        
        *ptr++ = (double) fid.positionMM.x;
        *ptr++ = (double) fid.positionMM.y;
        *ptr++ = (double) fid.positionMM.z;
        
        mxSetField (pMexTDF, (mwIndex) u, acTDF[0], pPOS); 
        mxSetField (pMexTDF, (mwIndex) u, acTDF[1], pEEP);
        mxSetField (pMexTDF, (mwIndex) u, acTDF[2], pTEMM);
    }
    return pMexTDF;
}

mxArray* frame2mat (const ftkFrameQuery& in)
{
    mxArray* pMexFrame = mxCreateStructMatrix(1,1,2, acFrame);
    
    if (in.threeDFiducialsStat == QS_OK)
    {
        mxArray* pTDF = tdf2mat (in.threeDFiducials, in.threeDFiducialsCount);
        mxSetField (pMexFrame, 0, acFrame[0], pTDF); 
    }
        
    if (in.markersStat == QS_OK)
    {
        mxArray* pMarkers = mrk2mat (in.markers, in.markersCount);
        mxSetField (pMexFrame, 0, acFrame[1], pMarkers);
    }
    
    return pMexFrame;
}

