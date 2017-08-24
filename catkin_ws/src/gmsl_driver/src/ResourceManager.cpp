/////////////////////////////////////////////////////////////////////////////////////////
// This code contains NVIDIA Confidential Information and is disclosed
// under the Mutual Non-Disclosure Agreement.
//
// Notice
// ALL NVIDIA DESIGN SPECIFICATIONS AND CODE ("MATERIALS") ARE PROVIDED "AS IS" NVIDIA MAKES
// NO REPRESENTATIONS, WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ANY IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
//
// NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. No third party distribution is allowed unless
// expressly authorized by NVIDIA.  Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2015-2016 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ConsoleColor.hpp"
#include "ResourceManager.hpp"

ResourceManager::~ResourceManager()
{
    releaseSAL();
    releaseDriveworks();
}

dwStatus ResourceManager::initDriveworks()
{
    dwStatus res = dwLogger_initialize(getConsoleLoggerCallback(true));
    if (DW_SUCCESS != res) return res;

    //dwLogger_setLogLevel(DW_LOG_WARN);
    dwLogger_setLogLevel(DW_LOG_ERROR);

    // instantiate Driveworks SDK context
    dwContextParameters sdkParams = dwContextParameters();

#ifdef VIBRANTE
    memset(&sdkParams, 0, sizeof(dwContextParameters));
#endif

    return dwInitialize(&m_SDKHandle, DW_VERSION, &sdkParams);
}

dwStatus ResourceManager::initSAL()
{
    return dwSAL_initialize(&m_salHandle, m_SDKHandle);
}

void ResourceManager::releaseSAL()
{
    dwSAL_release(&m_salHandle);
}

void ResourceManager::releaseDriveworks()
{
    dwRelease(&m_SDKHandle);
    dwLogger_release();
}

dwStatus ResourceManager::initializeResources(int argc,
                                              const char *argv[],
                                              ProgramArguments& arguments)
{
    try {
        //initSampleApp(argc, argv, arguments, 960, 604);
   	if (!arguments.parse(argc, argv))
        	exit(1); // Exit if not all require arguments are provided

    	std::string argumentString = arguments.printList();

	    if (argumentString.size() > 0)
        	std::cout << "Program Arguments:\n" << argumentString << std::endl;

    } catch (const std::runtime_error& e) {
        std::cerr << e.what();
        return DW_INTERNAL_ERROR;
    }

    dwStatus res = initDriveworks();
    if(DW_SUCCESS != res) {
        std::cerr << "Cannot initialize DriveWorks" << std::endl;
        return res;
    }

    //res = initRenderer();
    //if(DW_SUCCESS != res) {
    //    std::cerr << "Cannot initialize DriveWorks' renderer" << std::endl;
    //    return res;
    //}

    res = initSAL();
    if(DW_SUCCESS != res) {
        std::cerr << "Cannot initialize SAL" << std::endl;
        return res;
    }


    return DW_SUCCESS;
}

