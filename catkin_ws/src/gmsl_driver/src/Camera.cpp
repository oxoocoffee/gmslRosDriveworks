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

#include <Camera.hpp>
#include <iostream>
#include <cstring>

Camera::Camera(dwSensorHandle_t &sensor,
               dwSensorParams    salParams,
               dwSALHandle_t     sal,
               dwContextHandle_t sdk,
               ProgramArguments  arguments)
{
	// Init dwHandles
	this->sensor    = DW_NULL_HANDLE;

	//Initialize sensors
	dwStatus result = dwSAL_createSensor(&sensor, salParams, sal);

	// Initialize camera and image properties
	if (result == DW_SUCCESS)
    {
		this->sensor = sensor;

		dwSensorCamera_getImageProperties(&cameraImageProperties,
                            			   DW_CAMERA_PROCESSED_IMAGE,
                            			   this->sensor);

		dwSensorCamera_getSensorProperties(&cameraProperties, this->sensor);

		numSiblings = cameraProperties.siblings;

		std::cout << "Camera siblings: " << numSiblings <<  std::endl;
	}
	else {
        std::cerr << "Cannot create driver: " << salParams.protocol
                  << " with params: " << salParams.parameters << std::endl
                  << "Error: " << dwGetStatusName(result) << std::endl;

        if (result == DW_INVALID_ARGUMENT)
            std::cerr << "It is possible the given camera is not supported. " << std::endl;
	}

	initImagePool(sdk);
	initFormatter(sdk);
}

/* Initialize RGBA Image Pool for each Camera Port */

void Camera::initImagePool(dwContextHandle_t sdk)
{
    NvMediaDevice *nvmedia;
    dwContext_getNvMediaDevice(&nvmedia, sdk);

    for (int i = 0; i < numSiblings; ++i)
    {
        NvMediaImageAdvancedConfig advConfig;
        memset(&advConfig, 0, sizeof(advConfig));
        dwImageNvMedia*     rgbaImage = new dwImageNvMedia();
        NvMediaImage*       rgbaNvMediaImage;

        rgbaNvMediaImage = NvMediaImageCreate(nvmedia, NvMediaSurfaceType_Image_RGBA,
                                              NVMEDIA_IMAGE_CLASS_SINGLE_IMAGE, 1,
                                              cameraImageProperties.width,
                                              cameraImageProperties.height,
                                              0,
                                              &advConfig);

        dwImageNvMedia_setFromImage(rgbaImage, rgbaNvMediaImage);

        rgbaImagePool.push_back(rgbaImage);
    }

    std::cout << "Image pool created" << std::endl;
}

/* Initialize the image formatter */

void Camera::initFormatter(dwContextHandle_t sdk)
{
    dwImageProperties displayImageProperties = cameraImageProperties;
    displayImageProperties.pxlFormat         = DW_IMAGE_RGBA;
    displayImageProperties.planeCount        = 1;
    yuv2rgbaConverter                        = DW_NULL_HANDLE;

    dwStatus status                          = dwImageFormatConverter_initialize(&yuv2rgbaConverter,
                                                                                 &cameraImageProperties,
                                                                                 &displayImageProperties,
                                                                                 sdk);
    if (status != DW_SUCCESS)
    {
        std::cerr << "Cannot initialize pixel format converter" << std::endl;
        exit(1);
    }
    else
        std::cout << "Pixel format converter created" << std::endl;
}

/* Read frames from every camera on the csi-port and add them to the image pool */
/* Start camera and serialization */

bool Camera::start()
{
	return dwSensor_start(sensor) == DW_SUCCESS;
}

/* Stop the camera sensor and release all resources */
void Camera::stop_camera()
{
    dwSensor_stop(sensor);

    for (auto frame : rgbaImagePool)
    {
        NvMediaImageDestroy(frame->img);
        delete frame;
    }

    dwImageFormatConverter_release(&yuv2rgbaConverter);
}
