/*
 * Copyright (C) 2020 Arm Limited or its affiliates. All rights reserved.
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


#include "MainClassify.h"

#ifdef DNN
#include "DnnModel.h"

bool MainClassify::_InitModel()
{
	  
    
    //this->model = new Model();
  	this->model =  std::unique_ptr<DnnModel>(new DnnModel());
    //this->model =  std::make_unique<DnnModel>();  //C++14

    if (this->model) {
        return this->model->Init();
    }
    printf("Failed to allocate memory for the model\r\n");
    return false;
}
#endif
