#pragma once

#include <vector>

struct TransformationMatrix
{
    std::vector<double> translation;
    std::vector<std::vector<double>> rotation;
};