/**
 * Copyright (C) 2019 Yirami .
 * All rights reserved.
 * @file    hough_transform_mex_wrapper.cpp
 * @brief   Template class for Standard Hough Transform(SHT) .
 * @author  [Tang zhiyong](https://yirami.xyz)
 * @date    2019-06-28
 * @version 1.0
 * @note
 * @history
**/

#include "mex.hpp"
#include "mexAdapter.hpp"
#include "math.h"

using namespace matlab::data;
using matlab::mex::ArgumentList;

// #include "../include/hough_transform_wrapper.h"

class MexFunction : public matlab::mex::Function {
public:
    void operator()(ArgumentList outputs, ArgumentList inputs) {
        checkArguments(outputs, inputs);

        // // Get inputs
        // TypedArray<unsigned char> img_in = std::move(inputs[0]);
        // TypedArray<unsigned int> img_sz_in = std::move(inputs[1]);
        // TypedArray<unsigned int> max_lines_in = std::move(inputs[2]);
        // unsigned char *img = img_in.release().get();
        // unsigned int *img_sz = img_sz_in.release().get();
        // unsigned int *max_lines = max_lines_in.release().get();
        // // Creat Outputs
        // ArrayFactory factory;
        // double rho_max = sqrt(pow(img_sz[0]-1,2)+pow(img_sz[1]-1,2));
        // size_t rho_div = (size_t)floor(rho_max+0.5)*2+1;
        // TypedArray<size_t> vote = factory.createArray<size_t>({180, rho_div});
        // size_t *H = vote.release().get();
        // TypedArray<unsigned int> vote_sz = factory.createArray<unsigned int>({1, 2});
        // unsigned int *H_sz = vote_sz.release().get();
        // TypedArray<double> lines = factory.createArray<double>({6, *max_lines});
        // double *lines_mat = lines.release().get();
        // // CCall_SHT(img, img_sz, *max_lines, H, H_sz, lines_mat);
        // outputs[0] = vote;
        // outputs[1] = vote_sz;
        // outputs[2] = lines;
    }

    void checkArguments(ArgumentList outputs, ArgumentList inputs) {
        // // Get pointer to engine
        // std::shared_ptr<matlab::engine::MATLABEngine> matlabPtr = getEngine();
        //
        // // Get array factory
        // ArrayFactory factory;
        //
        // // Check for proper number of arguments
        // if (inputs.size() != 3) {
        //   matlabPtr->feval(u"error", 0, std::vector<Array>({ factory.createScalar("Three inputs required.") }));
        // }
        //
        // // Check input argument
        // if (inputs[0].getType() != ArrayType::UINT8 || inputs[0].getType() == ArrayType::COMPLEX_DOUBLE) {
        //     matlabPtr->feval(u"error", 0, std::vector<Array>({ factory.createScalar("First input must uint8") }));
        // }
        // if (inputs[1].getType() != ArrayType::UINT32 || inputs[1].getType() == ArrayType::COMPLEX_DOUBLE || inputs[1].getNumberOfElements() != 2) {
        //     matlabPtr->feval(u"error", 0, std::vector<Array>({ factory.createScalar("Second input must uint32 with two elements") }));
        // }
        // if (inputs[2].getType() != ArrayType::UINT32 || inputs[2].getType() == ArrayType::COMPLEX_DOUBLE || inputs[2].getNumberOfElements() != 1) {
        //     matlabPtr->feval(u"error", 0, std::vector<Array>({ factory.createScalar("Third input must uint32 scalar") }));
        // }
        // // Check number of outputs
        // if (outputs.size() != 3) {
        //     matlabPtr->feval(u"error", 0, std::vector<Array>({ factory.createScalar("Three outputs are returned") }));
        // }
    }
};
