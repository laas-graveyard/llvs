/*! ----------------------------------------------------
 *  Copyright 2010, CNRS-AIST JRL
 *
 *  \brief  Btl Slam for LLVS
 *  \author Clement Petit
 *  \creation 24/06/2010
 * ---------------------------------------------------- */

#include "BtlSlam/BtlSlamProcess.h"

#if (LLVS_HAVE_HRP_BTL_SLAM>0)

/*! Includes system */
#include <sstream>
#include <stdio.h> //memcpy
#include <fstream> //to be removed

/*! Includes slam specific */
#include <vslam_app.h>

/*! Includes llvs tools */
#include <llvs/tools/Debug.h>

/*! Parameters */
//FIXME: Remove those hard-coded values
#define BTL_SLAM_CAMERA_WIDTH	      320
#define BTL_SLAM_CAMERA_HEIGHT      240
#define BTL_SLAM_CAMERA_DEPTH       3
#define BTL_SLAM_SHARED_MEMORY_SIZE 65536
#define BTL_SLAM_RGB_SIZE           "255"
#define BTL_SLAM_PPM_FORMAT         "P6"

/* ---------------------------------------------------
 * Initialization / Destruction
 * --------------------------------------------------- */

HRP2BtlSlamProcess::HRP2BtlSlamProcess()
	:m_pImageContainer(0),
	m_pSharedBuffer(0),
	m_pSharedSegment(0),
	m_isAlreadyStarted(false)
{
	m_slamConfig.options = NULL;
	m_slamConfig.size = 0;
	m_ProcessName = "BtlSlamProcess";
}

HRP2BtlSlamProcess::~HRP2BtlSlamProcess()
{
	pStopProcess();
	pCleanUpTheProcess();
}


/* ---------------------------------------------------
 * Generic interface
 * --------------------------------------------------- */

int
HRP2BtlSlamProcess::pInitializeTheProcess()
{
	ODEBUG3("[BtlSlam] Initialize");

	// Look if configuration struct is ready
	if( m_slamConfig.size == 0 )
	{
		return BTL_SLAM_ERROR_CONFIG_MISSING;
	}
	ODEBUG3("parameters:" << m_slamConfig.size);
	for(int i = 0; i < m_slamConfig.size; ++i)
	{
		ODEBUG3(i << ":" << m_slamConfig.options[ i ]);
	}
	ODEBUG3("SERVER:" << m_slamConfig.server);
	ODEBUG3("CAMERA:" << m_slamConfig.camera);
	ODEBUG3("----------");

	// Create a shared memory block
	cleanUpSharedMemory();
	try
	{
		m_pSharedSegment = new boost::interprocess::managed_shared_memory(
				boost::interprocess::create_only,
				m_slamConfig.server.c_str(),
				BTL_SLAM_SHARED_MEMORY_SIZE
				);
	}
	catch (boost::interprocess::interprocess_exception &ex)
	{
		ODEBUG3("[BtlSlam] Segment read error : " << ex.what() );
		throw(ex);
	}
	m_pSharedBuffer = m_pSharedSegment->construct<CVD::LLVSBuffer>(
			m_slamConfig.camera.c_str()
			)();

	// Push an image
	{
		boost::interprocess::scoped_lock<boost::interprocess::interprocess_recursive_mutex> lock(m_pSharedBuffer->mutex);
		int imageSize = BTL_SLAM_CAMERA_HEIGHT * BTL_SLAM_CAMERA_WIDTH * BTL_SLAM_CAMERA_DEPTH;
		m_pSharedBuffer->image = new unsigned char[imageSize];
		std::memcpy(
				m_pSharedBuffer->image,
				*m_pImageContainer,
				imageSize
				);
	}

	// Initialize the slam engine
	try
	{
		VSLAM_App::Instance().init(
				m_slamConfig.size,
				m_slamConfig.options,
				true
				);
	}
	catch(VSLAM_Common::Exception ex)
	{
		ODEBUG3("Exception caught : " << ex.msg);
		ODEBUG3("Error code : " << ex.code);
		VSLAM_App::Instance().exit();
		return BTL_SLAM_ERROR_INITIALIZATION_FAILED;
	}

	return BTL_SLAM_RESULT_OK;
}

int
HRP2BtlSlamProcess::pRealizeTheProcess()
{
	ODEBUG3("[BtlSlam] Realize");
	return BTL_SLAM_RESULT_OK;
}

int
HRP2BtlSlamProcess::pCleanUpTheProcess()
{
	ODEBUG3("[BtlSlam] CleanUp");

	// Remove shared memory
	cleanUpSharedMemory();

	// Clear configuration variable
	cleanUpConfig();

	return BTL_SLAM_RESULT_OK;
}

int
HRP2BtlSlamProcess::pStartProcess()
{
	ODEBUG3("[BtlSlam] Launch initialization");
	if( m_isAlreadyStarted )
	{
		ODEBUG3("[BtlSlam] Process has already been started once.");
		return BTL_SLAM_ERROR_PROCESS_ALREADY_STARTED;
	}
	int result( pInitializeTheProcess() );
	if( result != BTL_SLAM_RESULT_OK )
	{
		ODEBUG3("[BtlSlam] Error code " << result);
		return result;
	}
	ODEBUG3("[BtlSlam] Start");
	m_isAlreadyStarted = true;
	VSLAM_App::Instance().start();
	return BTL_SLAM_RESULT_OK;
}

int
HRP2BtlSlamProcess::pStopProcess()
{
	m_isAlreadyStarted = false;
	ODEBUG3("[BtlSlam] Stop");
	return BTL_SLAM_RESULT_OK;
}

int
HRP2BtlSlamProcess::pSetParameter(std::string aParameter, std::string aValue)
{
	ODEBUG3("[BtlSlam] Set a parameter :" << aParameter);
	if(	aParameter == "config" )
	{
		return setSlamConfig(aValue);
	}
	else
	{
		ODEBUG3("[BtlSlam] Unknown parameter");
		return BTL_SLAM_ERROR_UNKNOWN_PARAMETER;
	}
}

/* ---------------------------------------------------
 * Specific interface
 * --------------------------------------------------- */

void
HRP2BtlSlamProcess::SetInputImages(unsigned char** pImageContainer)
{
	ODEBUG3("[BtlSlam] Set image buffer address: " << (void*)pImageContainer
			<< " currently pointing " << (void*)*pImageContainer);
	m_pImageContainer = pImageContainer;
}

int
HRP2BtlSlamProcess::setSlamConfig(const std::string& config)
{
	std::string buffer;

	// Clean previous config
	cleanUpConfig();

	// Look a first time the string to find out the number
	// of elements
	std::stringstream stream(config);
	m_slamConfig.size = 0;
	for(m_slamConfig.size = 0; !stream.eof(); ++m_slamConfig.size)
	{
		stream >> buffer;
	}
	if( m_slamConfig.size == 0 )
	{
		return BTL_SLAM_ERROR_CONFIG_MISSING;
	}

	// Then create config object
	m_slamConfig.options = new char*[m_slamConfig.size];

	// Rewind stream and fill the config var
	size_t found;
	stream.clear();
	stream.seekg(0, std::ios::beg);
	for(int i = 0; i < m_slamConfig.size; ++i)
	{
		stream >> buffer;
		m_slamConfig.options[ i ] = new char[buffer.size() + 1];
		buffer.copy(m_slamConfig.options[ i ], buffer.size());
		m_slamConfig.options[ i ][ buffer.size() ] = '\0';

		// Look up for shared memory location
		found = buffer.find("shared:");
		if( found != std::string::npos )
		{
			size_t found_server;
			found = buffer.rfind(".");
			found_server = buffer.rfind("/");
			if(   (found == std::string::npos)
					||(found_server == std::string::npos)
					||(found - found_server - 1 <= 0))
			{
				m_slamConfig.size = i + 1;
				cleanUpConfig();
				return BTL_SLAM_ERROR_BAD_SOURCE_SYNTAX;
			}
			else
			{
				m_slamConfig.camera = buffer.substr(found + 1);
				m_slamConfig.server = buffer.substr(found_server + 1,
				                                   found - found_server - 1);
			}
		}
	}

	return BTL_SLAM_RESULT_OK;
}

/* ---------------------------------------------------
 * For debug
 * --------------------------------------------------- */

bool
HRP2BtlSlamProcess::writeImageIntoFile(const unsigned char* rgbFrame,
                                       const std::string& filename)
const
{
	// Write the image on disk for check
	std::ofstream aofstream(filename.c_str(), std::ofstream::out);
	if (!aofstream.is_open())
	{
		return false;
	}
	unsigned int imageSize =  BTL_SLAM_CAMERA_WIDTH
						                * BTL_SLAM_CAMERA_HEIGHT
								            * BTL_SLAM_CAMERA_DEPTH;
	aofstream << BTL_SLAM_PPM_FORMAT << std::endl;
	aofstream << BTL_SLAM_CAMERA_WIDTH << " "
						<< BTL_SLAM_CAMERA_HEIGHT << std::endl;
	aofstream << BTL_SLAM_RGB_SIZE <<std::endl;
	for(unsigned int i=0; i < imageSize; ++i)
	{
		aofstream << rgbFrame[ i ];
	}
	aofstream.close();
	return true;
}


/* ---------------------------------------------------
 * Atomic clean up methods
 * --------------------------------------------------- */

void
HRP2BtlSlamProcess::cleanUpConfig()
{
	if( m_slamConfig.options != NULL )
	{
		for( int i = 0; i < m_slamConfig.size; ++i)
		{
			delete m_slamConfig.options[ i ];
		}
		delete [] m_slamConfig.options;
		m_slamConfig.options = NULL;
		m_slamConfig.size = 0;
	}
}

void
HRP2BtlSlamProcess::cleanUpSharedMemory()
{
	if( m_pSharedSegment )
	{
		m_pSharedSegment->destroy<CVD::LLVSBuffer>(m_slamConfig.camera.c_str());
		delete m_pSharedSegment;
	}
	boost::interprocess::shared_memory_object::remove(m_slamConfig.server.c_str());
}


#endif // LLVS_HAVE_HRP_BTL_SLAM
