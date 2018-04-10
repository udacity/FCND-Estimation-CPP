#pragma once

// "Sigma threshold" trigger/detector
// Detects if the value of a signal is within a certain other signal/constant of a reference signal/constant

#include "BaseAnalyzer.h"
#include "Utility/StringUtils.h"
#include "Utility/FixedQueue.h"

using std::string;

class SigmaThreshold : public BaseAnalyzer
{
public:
  SigmaThreshold(string var, string ref, string sigma, float minPassThresh, float maxPassThresh, float minTimeWindow)
		: low(MAX_POINTS, 0), high(MAX_POINTS, 0), x(MAX_POINTS, 0)
  {
    _var = var;
		_ref = _sigma = "";

		if (SLR::HasLetters(ref))
		{
			_ref = ref;
		}
		else
		{
			_constRef = (float)atof(ref.c_str());
		}

		if (SLR::HasLetters(sigma))
		{
			_ref = _ref;
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

	void Reset()
	{
		if (_lastTime != 0)
		{
			/*if (_active)
			{
				printf("PASS: ABS(%s) was less than %lf for at least %lf seconds\n", _var.c_str(), _thresh, _minWindow);
			}
			else
			{
				printf("FAIL: ABS(%s) was less than %lf for %lf seconds, which was less than %lf seconds\n", _var.c_str(), _thresh, _lastTime- _lastTimeAboveThresh, _minWindow);
			}*/ // TODO
		}
		_lastViolationTime = numeric_limits<float>::infinity();
		if (_ref == "")
		{
			_lastRefVal = _constRef;
		}
		else
		{
			_lastRefVal = numeric_limits<float>::infinity();
		}

		if (_sigma == "")
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
		// only run the full thing if we have a new measurement

		_lastTime = (float)time;

		if (_lastViolationTime == numeric_limits<float>::infinity())
		{
			_lastViolationTime = (float)time;
		}


		if (_ref != "")
		{
			TryUpdate(sources, _ref, _lastRefVal);
		}
		if (_sigma != "")
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

  }

  void OnNewData(float time, float meas)
  {
    if (_active)
    {
      return;
    }




    /*if (fabs(meas) > _thresh)
    {
			_lastViolationTime = time;
    }

    if ((time - _lastTimeAboveThresh) > _minWindow)
    {
      _active = true;
    }*/ // TODO
		// form a +/- series?
  }

  // Draws horizontal threshold bands
  // and detection marker/time
  void Draw(float minX, float maxX, float minY, float maxY)
  {
    glColor3f(.1f, .2f, .1f);

		if (x.n_meas() < 2) return;


		glColor3f(.8f, .8f, .8f);

		glEnable(GL_LINE_STIPPLE);
		glLineStipple(5, 0xAAAA);

		glBegin(GL_LINE_STRIP);
		for (unsigned int i = 0; i < x.n_meas(); i++)
		{
			glVertex2f(x[i], low[i]);
		}
		glEnd();
		glBegin(GL_LINE_STRIP);
		for (unsigned int i = 0; i < x.n_meas(); i++)
		{
			glVertex2f(x[i], high[i]);
		}
		glEnd();

		glDisable(GL_LINE_STIPPLE);

		float per = (float)in / (float)(in + out)*100.f;

		glColor3f(.8f, .8f, .8f);
		char buf[100];
		sprintf_s(buf, 100, "%.0lf", per);
		DrawStrokeText(buf, _lastTime - (maxX - minX)*.05f, minY + (maxY - minY) / 2.f, 0, 1.2f, (maxX - minX) / 2.f, (maxY - minY) / 2.f *2.f);
  }

  bool _active;

	float _threshMin, _threshMax;
	float maxPassThresh;
	float _minTimeWindow;
	float _lastTime;
	float _lastViolationTime;

	string _var, _ref, _sigma;
	float _constSigma;
	float _constRef;

	float _lastRefVal, _lastSigmaVal;

	FixedQueue<float> low, high, x;

	int in, out;
};