#pragma once

#include <string>
#include "dart/dart.h"
#include "dist-json/json/json.h"
// #include "util/MathUtil.h"

class cJsonUtil
{
public:
	static std::string BuildVectorJson(const Eigen::VectorXd& vec);
	static bool ReadVectorJson(const Json::Value& root, Eigen::VectorXd& out_vec);
	static bool ReadMatrixJson(const Json::Value& root, Eigen::MatrixXd& out_mat);

private:
	
};