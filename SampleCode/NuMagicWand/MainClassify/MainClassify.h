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

#ifndef __MainClassify_H__
#define __MainClassify_H__

#include <vector>

#include "Model.h"
#include "MPU6500.h"

class MainClassify {

public:
    MainClassify(uint32_t nElements);

    ~MainClassify() = default;

    void ExtractFeatures();
    void Classify();
    void AveragePredictions();
    int GetTopClass(const std::vector<float>& prediction);
    uint8_t FillSensorData(void);

    //std::vector<int16_t> audioBuffer;
    //std::vector<float> mfccBuffer;
    //std::vector<float> gsensorBuffer;
    float gsensorBuffer[128*3];
    std::vector<float> output;
    //std::vector<float> predictions;
    //std::vector<float> averagedOutput;
   
    int numOutClasses;
    int numInputDims[3];
    //int audioBlockSize;
    //int audioBufferSize;
		uint16_t gsensorBufferIdx;

protected:
    /** @brief Initialises the model */
    bool _InitModel();
    void InitMainClassify();
    //std::unique_ptr<MFCC> mfcc;
    //MFCC* mfcc; 
    std::unique_ptr<Model> model;
};

#endif /* __MainClassify_H__ */
