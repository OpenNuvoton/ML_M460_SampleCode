/*
 * Copyright (C) 2021 Arm Limited or its affiliates. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description: Keyword spotting example code using MFCC feature extraction
 * and neural network.
 */

#include "MainClassify.h"
#include <float.h>

MainClassify::MainClassify(uint32_t nElements)
{
    if (this->_InitModel()) {
				this->gsensorBufferIdx = 0;
        this->InitMainClassify();
    }
}

void MainClassify::InitMainClassify()
{
 	  if (!model->IsInited()) {
        printf("Warning: model has not been initialised\r\n");
        model->Init();
    }

		numInputDims[0] = model->GetInputShape()->data[0]; //Check input data  1x128x3 
		numInputDims[1] = model->GetInputShape()->data[1];
		numInputDims[2] = model->GetInputShape()->data[2];
    numOutClasses = model->GetOutputShape()->data[1];  // Output shape should be [1, numOutClasses].

    // Following are for debug purposes.
    printf("Initialising MAGICWAND object..\r\n");
		printf("numInputDims: %d-%d-%d\r\n", numInputDims[0], numInputDims[1], numInputDims[2]);
	  printf("numOutClasses: %d\r\n", numOutClasses);

    output = std::vector<float>(numOutClasses, 0.0);
  
}


void MainClassify::Classify()
{
   
    float* inTensorData = tflite::GetTensorData<float>(model->GetInputTensor());
	
	  // Copy -sensor data into the TfLite tensor.
    memcpy(inTensorData, gsensorBuffer, 128*3 * sizeof(float));
	  
    // Run inference on this data.
    model->RunInference();

    // Get output from the TfLite tensor.
    float* outTensorData = tflite::GetTensorData<float>(model->GetOutputTensor());
    memcpy(output.data(), outTensorData, numOutClasses * sizeof(float));
}

uint8_t MainClassify::FillSensorData(void)
{
      float fBuff[3];
	    uint8_t ret=0;
	    if(gsensorBufferIdx < 128*3)
      {
		         MPU6500_readXYZ_mg(fBuff);
					    this->gsensorBuffer[gsensorBufferIdx] = fBuff[0];
	            this->gsensorBuffer[gsensorBufferIdx+1] =  fBuff[1];
	            this->gsensorBuffer[gsensorBufferIdx+2] =  fBuff[2];
			  		 gsensorBufferIdx+=3;
		  }
	    else
      {
	            gsensorBufferIdx = 0;
				      ret =1;
      }
     return ret;
}

int MainClassify::GetTopClass(const std::vector<float>& prediction)
{
    int maxInd = 0;
    float maxVal = FLT_MIN;
    for (int i = 0; i < numOutClasses; i++) {
        if (maxVal < prediction[i]) {
            maxVal = prediction[i];
            maxInd = i;
        }
    }
    return maxInd;
}



