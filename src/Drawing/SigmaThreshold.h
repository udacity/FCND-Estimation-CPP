#pragma once

// "Sigma threshold" trigger/detector
// Detects if the value of a signal is within a certain other signal/constant of a reference signal/constant

#include "BaseAnalyzer.h"
#include "Utility/StringUtils.h"
#include "Utility/FixedQueue.h"
#include "Utility/SimpleConfig.h"

using namespace SLR;
using std::string;

class SigmaThreshold : public BaseAnalyzer
{
public:
  SigmaThreshold(string var, string ref, string sigma, float minPassThresh, float maxPassThresh, float minTimeWindow)
		: low(MAX_POINTS, 0), high(MAX_POINTS, 0), x(MAX_POINTS, 0)
  {
    _var = var;
		_ref = _sigma = "";

		_constRef = _constSigma = numeric_limits<float>::infinity();

		if (SLR::HasLetters(ref))
		{
			_ref = SLR::Trim(ref);
		}
		else
		{
			_constRef = (float)atof(ref.c_str());
		}

		if (SLR::HasLetters(sigma))
		{
			_sigma = SLR::Trim(sigma);
		}
		else
		{
			_constSigma = (float)atof(sigma.c_str());
		}

		_threshMin = minPassThresh;
		_threshMax = maxPassThresh;
		_minTimeWindow = minTimeWindow;

    _lastTime = 0;
    Reset();
  }

	string MakeStringFromParamOrConst(string param, float c)
	{
		string ret = param;
		if (param== "")
		{
			char buf[100];
			sprintf_s(buf, 100, "%lf", c);
			ret = buf;
		}
		return ret;
	}

	void Reset()
	{
		if (_lastTime != 0)
		{
			string refS = MakeStringFromParamOrConst(_ref, _constRef);
			string sigmaS = MakeStringFromParamOrConst(_sigma, _constSigma);
			float per = (float)in / (float)(in + out)*100.f;
			if ((_lastTime - _lastViolationTime) >= _minTimeWindow)
			{				
				printf("PASS: ABS(%s-%s) was less than %s for %.0lf%% of the time\n", _var.c_str(), refS.c_str(), sigmaS.c_str(), per);
			}
			else
			{
				printf("FAIL: ABS(%s-%s) was less than %s for %.0lf%% of the time\n", _var.c_str(), refS.c_str(), sigmaS.c_str(), per);
			}
		}

		ParamsHandle paramSys = SimpleConfig::GetInstance();

		_lastViolationTime = numeric_limits<float>::infinity();
		if (_ref == "" || paramSys->GetFloat(_ref,_constRef))
		{
			_lastRefVal = _constRef;
		}
		else
		{
			_lastRefVal = numeric_limits<float>::infinity();
		}

		if (_sigma == "" || paramSys->GetFloat(_sigma, _constSigma))
		{
			_lastSigmaVal = _constSigma;
		}
		else
		{
			_lastSigmaVal = numeric_limits<float>::infinity();
		}
		
    _active = false;
		low.reset();
		high.reset();
		x.reset();

		in = 0;
		out = 0;
  }

	bool TryUpdate(std::vector<shared_ptr<DataSource> >& sources, string& varname, float& ret)
	{
		for (unsigned int j = 0; j < sources.size(); j++)
		{
			if (sources[j]->GetData(varname, ret))
			{
				return true;
			}
		}
		return false;
	}

  void Update(double time, std::vector<shared_ptr<DataSource> >& sources)
  {
		float tmp = 0;
		if (!TryUpdate(sources, _var, tmp))
		{
			return;
		}

		// we only run the full thing if we have a new measurement
		_lastTime = (float)time;

		if (_lastViolationTime == numeric_limits<float>::infinity())
		{
			_lastViolationTime = (float)time;
		}

		if (_constRef == numeric_limits<float>::infinity())
		{
			TryUpdate(sources, _ref, _lastRefVal);
		}
		if (_constSigma == numeric_limits<float>::infinity())
		{
			TryUpdate(sources, _sigma, _lastSigmaVal);
		}

		low.push(_lastRefVal - _lastSigmaVal);
		high.push(_lastRefVal + _lastSigmaVal);
		x.push((float)time);

		if (tmp >= (_lastRefVal - _lastSigmaVal) && tmp <= (_lastRefVal + _lastSigmaVal))
		{
			in++;
		}
		else
		{
			out++;
		}

		float per = (float)in / (float)(in + out)*100.f;
		if(per <= _threshMin || per >=_threshMax)
		{
			_lastViolationTime = (float)time;
		}
		

  }

  // Draws horizontal threshold bands
  // and detection marker/time
  void Draw(float minX, float maxX, float minY, float maxY)
  {
    glColor3f(.1f, .2f, .1f);

		if (x.n_meas() < 2) return;

		float g = 0.8f;
		if ((_lastTime- _lastViolationTime) >= _minTimeWindow)
		{
			g = 1.f;
			glColor3f(.2f, g, .2f);
		}
		else
		{
			glColor3f(.7f, .7f, .7f);
		}

		glBegin(GL_LINE_STRIP);
		for (unsigned int i = 0; i < x.n_meas(); i++)
		{
			glVertex2f(x[i], CONSTRAIN(low[i],minY,maxY));
		}
		glEnd();
		glBegin(GL_LINE_STRIP);
		for (unsigned int i = 0; i < x.n_meas(); i++)
		{
			glVertex2f(x[i], CONSTRAIN(high[i],minY,maxY));
		}
		glEnd();

		float per = (float)in / (float)(in + out)*100.f;

		const float dx = maxX - minX;
		const float dy = maxY - minY;
		const float left = minX + dx*.1f;
		const float bot = minY + dy / 2.f;

		glColor4f(.8f, g, .8f, .8f);
		glBegin(GL_QUADS);
		glVertex2f(left -dx*.02f, bot-dy*.02f);
		glVertex2f(left +dx*.07f, bot - dy * .02f);
		glVertex2f(left +dx*.07f, bot+dy*.1f);
		glVertex2f(left - dx * .02f, bot+dy*.1f);
		glEnd();

		glColor3f(.1f, .1f, .1f);
		char buf[100];
		sprintf_s(buf, 100, "%.0lf%%", per);
		DrawStrokeText(buf, left, bot, 0, 1.2f, (maxX - minX) / 2.5f, (maxY - minY) / 2.5f *2.f);
  }

  bool _active;

	float _threshMin, _threshMax;
	float maxPassThresh;
	float _minTimeWindow;
	float _lastTime;
	float _lastViolationTime;

	string _var, _ref, _sigma;

	// for each of these, if the value is inifinity, then it's a dynamic value that should be read from the datasource system
	float _constSigma;
	float _constRef;

	float _lastRefVal, _lastSigmaVal;

	FixedQueue<float> low, high, x;

	int in, out;
};