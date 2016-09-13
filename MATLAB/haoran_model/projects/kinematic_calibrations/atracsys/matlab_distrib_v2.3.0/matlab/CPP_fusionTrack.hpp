#ifndef CPP_fusionTrack_hpp
#define CPP_fusionTrack_hpp

	#include <ftkInterface.h>
	#include <vector>
    #include "CPP_geometry.hpp"
	
	#define DISPLAY_ERROR(err,val,msg) \
		case err: mexErrMsgTxt ("fusionTrack error " #err); break;
		
	class fusionTrack
	{
	private:
		static void devicesCallback (uint64 sn, void* user, ftkDeviceType type)
		{
			if (!user)
				return;
			std::vector<uint64>* vec =  (std::vector<uint64>*) user;
			vec->push_back (sn);
		}
        
        static void geometriesCallback (uint64 sn, void* user, ftkGeometry* in)
        {
			if (!user)
				return;
			std::vector<ftkGeometry>* vec =  (std::vector<ftkGeometry>*) user;
			vec->push_back (*in);
        }
        
        static void optionsCallback (uint64 sn, void* user, ftkOptionsInfo* in)
        {
			if (!user)
				return;
			std::vector<ftkOptionsInfo>* vec =  (std::vector<ftkOptionsInfo>*) user;
			vec->push_back (*in);
        }
	
    public:
        
        static const unsigned cstMax3DFiducials = 256;
        static const unsigned cstMaxMarkers = 16;
        
	protected:
		ftkLibrary handle;
        ftkFrameQuery* frame;
        
        ftkMarker markers [cstMaxMarkers];
        ftk3DFiducial fiducials [cstMax3DFiducials];
		
		void check (ftkError err)
		{
			if (err > 0) // if (err != 0)
				switch (err)
				{
					ERRORS(DISPLAY_ERROR)
				}
		}
		
	public:
        
       
		fusionTrack (): handle (NULL) 
		{
			handle = ftkInit ();
            
			frame = ftkCreateFrame();
			ftkSetFrameOptions( false, false, 0u, 0u, 256u, 16u, frame );
        }
		
		virtual ~fusionTrack ()
		{
			check (ftkClose (handle));
			ftkDeleteFrame( frame );
		}
		
		std::vector<uint64> devices ()
		{
			std::vector<uint64> vec;
			check (ftkEnumerateDevices (handle, devicesCallback, &vec));
			return vec;
		}
        
        std::vector<ftkGeometry> geometries (uint64 device)
		{
			std::vector<ftkGeometry> vec;
			check (ftkEnumerateGeometries (handle, device, geometriesCallback, &vec));
			return vec;
		}
        
        void clearGeometry (uint64 device, uint32 id)
        {
            check (ftkClearGeometry (handle, device, id));
        }
        
        void setGeometry (uint64 device, ftkGeometry& in)
        {
            check (ftkSetGeometry (handle, device, &in));
        }
        
        std::vector<ftkOptionsInfo> options (uint64 device)
		{
			std::vector<ftkOptionsInfo> vec;
			check (ftkEnumerateOptions (handle, device, optionsCallback, &vec));
			return vec;
		}
        
        int32 getInt32 (uint64 device, uint32 optId, ftkOptionGetter what)
        {
            int32 out;
            check (ftkGetInt32 (handle, device, optId, &out, what));
            return out;
        }
        
        void setInt32 (uint64 device, uint32 optId, int32 val)
        {
            check (ftkSetInt32 (handle, device, optId, val));
        }
        
        float32 getFloat32 (uint64 device, uint32 optId, ftkOptionGetter what)
        {
            float32 out;
            check (ftkGetFloat32 (handle, device, optId, &out, what));
            return out;
        }
        
        void setFloat32 (uint64 device, uint32 optId, float32 val)
        {
            check (ftkSetFloat32 (handle, device, optId, val));
        }

        void getLastFrame (uint64 device, uint32 timeout = 100) // Bug with zero ?!?
        {
            check (ftkGetLastFrame (handle, device, frame, timeout));
        }

        const ftkFrameQuery& getFrame () const { return *frame; };
    };

#endif