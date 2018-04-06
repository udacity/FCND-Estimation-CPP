#pragma once

#include <map>
#include <vector>
#include "matrix/math.hpp"
using std::vector;
using std::map;

#include "Eigen/Dense"
using Eigen::MatrixXf;
using Eigen::VectorXf;


namespace SLR{

class SimpleConfig;
typedef shared_ptr<SimpleConfig> ParamsHandle;


class SimpleConfig
{
	SimpleConfig();
	void ReadFile(const string& filename, int depth=0);

public:
	static ParamsHandle GetInstance();
	void Reset(string rootParam);
  
  bool Exists(const string& param);
  bool GetFloat(const string& param, float& ret);
  bool GetString(const string& param, string& ret);
  bool GetV3F(const string& param, V3F& ret);
  bool GetFloatVector(const string& param, vector<float>& ret);
  
  template<size_t N>
  inline bool GetFloatVector(const string& param, matrix::Vector<float, N>& ret)
  {
    vector<float> tmp;
    if (!GetFloatVector(param, tmp)) return false;
    if (tmp.size() != N) return false;
    for (size_t i = 0; i < N; i++)
    {
      ret(i) = tmp[i];
    }
    return true;
  }

	inline bool GetFloatVector(const string& param, VectorXf& ret)
	{
		vector<float> tmp;
		if (!GetFloatVector(param, tmp)) return false;
		ret.resize(tmp.size());
		for (size_t i = 0; i < tmp.size(); i++)
		{
			ret(i) = tmp[i];
		}
		return true;
	}

  // convenience always-returning functions, with defaults
  float Get(const string& param, float defaultRet);
  string Get(const string& param, string defaultRet);
  V3F Get(const string& param, V3F defaultRet);

  void PrintAll();

protected:
	static shared_ptr<SimpleConfig> s_config;
  map<string,string> _params;
  void ParseLine(const string& filename, const string& ln, int lineNum, string& curNamespace, int depth);
  void CopyNamespaceParams(const string& fromNamespace, const string& toNamespace);
};



} // namespace SLR
