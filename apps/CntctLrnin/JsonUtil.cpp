#include "JsonUtil.h"

std::string cJsonUtil::BuildVectorJson(const Eigen::VectorXd& vec)
{
	std::string json = "";
	for (int i = 0; i < vec.size(); ++i)
	{
		if (i != 0)
		{
			json += ", ";
		}
		json += std::to_string(vec[i]);
	}
	json = "[" + json + "]";
	return json;
}

bool cJsonUtil::ReadVectorJson(const Json::Value& root, Eigen::VectorXd& out_vec)
{
	bool succ = false;
	int num_vals = root.size();
	
	if (root.isArray())
	{
		out_vec.resize(num_vals);
		for (int i = 0; i < num_vals; ++i)
		{
			Json::Value json_elem = root.get(i, 0);
			out_vec[i] = json_elem.asDouble();
		}
		succ = true;
	}

	return succ;
}


bool cJsonUtil::ReadMatrixJson(const Json::Value& root, Eigen::MatrixXd& out_mat)
{
	bool succ = false;
	int num_rows = root.size();
	assert(num_rows > 0);
	int num_cols = root[0].size();
	
	if (root.isArray())
	{
		out_mat.resize(num_rows,num_cols);
		for (int i = 0; i < num_rows; ++i)
		{
			Json::Value json_row = root[i];
			for (int j = 0; j < num_cols; ++j)
			{
				Json::Value json_elem = json_row[j];
				out_mat(i,j) = json_elem.asDouble();
			}			
		}
		succ = true;
	}
	
	
	return succ;
}
