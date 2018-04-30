#include "Common.h"
#include "Graph.h"
#include "Drawing/DrawingFuncs.h"
#include "Utility/SimpleConfig.h"
#include "Utility/StringUtils.h"
#include "DataSource.h"
#include "ColorUtils.h"
#include "Drawing/AbsThreshold.h"
#include "Drawing/WindowThreshold.h"
#include "Drawing/SigmaThreshold.h"

using namespace SLR;

Graph::Graph(const char* name)
{
  _name = name;
	_logFile = NULL;
  Reset();
}

Graph::Series::Series()
  : x(MAX_POINTS, 0), y(MAX_POINTS, 0)
{
  noLegend = false;
  bold = false;
  negate = false;
}

void Graph::AddItem(string path)
{
  if (path.find("AbsThreshold(") != string::npos)
  {
    AddAbsThreshold(path.substr(12));
  }
  else if (path.find("WindowThreshold(") != string::npos)
  {
    AddWindowThreshold(path.substr(15));
  }
	else if (path.find("SigmaThreshold(") != string::npos)
	{
		AddSigmaThreshold(path.substr(14));
	}
	else if (ToUpper(path) == "LOGTOFILE")
	{
		BeginLogToFile();
	}
  else if (path.find("SetYAxis(") != string::npos)
  {
    SetYAxis(path.substr(9));
  }
  else
  {
    AddSeries(path);
  }
}

void Graph::AddItem(string path, vector<string> options)
{

}

void Graph::AddAbsThreshold(string path)
{
  path = SLR::Trim(path);
  if (path.length() < 4 || path[0] != '(' || path[path.length() - 1] != ')')
  {
    SLR_WARNING1("Malformed AbsThreshold command (%s)", path.c_str());
    return;
  }
  path = path.substr(1, path.length() - 2);

  vector<string> args = SLR::Split(path, ',');

  if (args.size()!=3 || args[0]=="" || args[1]=="" || args[2]=="")
  {
    SLR_WARNING1("Malformed AbsThreshold command (%s)", path.c_str());
    return;
  }

  shared_ptr<AbsThreshold> thr(new AbsThreshold(args[0], (float)atof(args[1].c_str()), (float)atof(args[2].c_str())));
  _analyzers.push_back(thr);
}

void Graph::SetYAxis(string argsString)
{
  vector<string> args = SLR::Split(argsString, ',');
  if (args.size() != 2)
  {
    SLR_WARNING1("Malformed SetYAxis command (%s)", argsString.c_str());
  }
  
  _graphYLow = (float)atof(args[0].c_str());
  _graphYHigh = (float)atof(args[1].c_str());
}

void Graph::AddWindowThreshold(string path)
{
  path = SLR::Trim(path);
  if (path.length() < 4 || path[0] != '(' || path[path.length() - 1] != ')')
  {
    SLR_WARNING1("Malformed WindowThreshold command (%s)", path.c_str());
    return;
  }
  path = path.substr(1, path.length() - 2);

  vector<string> args = SLR::Split(path, ',');

  if (args.size() != 3 || args[0] == "" || args[1] == "" || args[2] == "")
  {
    SLR_WARNING1("Malformed WindowThreshold command (%s)", path.c_str());
    return;
  }

  shared_ptr<WindowThreshold> thr(new WindowThreshold(args[0], (float)atof(args[1].c_str()), (float)atof(args[2].c_str())));
  _analyzers.push_back(thr);
}

void Graph::AddSigmaThreshold(string path)
{
	path = SLR::Trim(path);
	if (path.length() < 4 || path[0] != '(' || path[path.length() - 1] != ')')
	{
		SLR_WARNING1("Malformed SigmaThreshold command (%s)", path.c_str());
		return;
	}
	path = path.substr(1, path.length() - 2);

	vector<string> args = SLR::Split(path, ',');

	if (args.size() != 6|| args[0] == "" || args[1] == "" || args[2] == "")
	{
		SLR_WARNING1("Malformed SigmaThreshold command (%s)", path.c_str());
		return;
	}

	shared_ptr<SigmaThreshold> thr(new SigmaThreshold(args[0], args[1],args[2],
		(float)atof(args[3].c_str()),
		(float)atof(args[4].c_str()), 
		(float)atof(args[5].c_str())
	));
	_analyzers.push_back(thr);
}

void Graph::AddSeries(string path, bool autoColor, V3F color, vector<string> options)
{
	ParamsHandle config = SimpleConfig::GetInstance();

  Series newSeries;
  bool force = false;

  if (path.find(',') != string::npos)
  {
    options = SLR::Split(path, ',');
    path = options[0];
  }
  int colorNum =0;

  newSeries._legend = path;
    
  for (size_t i = 1; i < options.size(); i++)
  {
    options[i] = Trim(options[i]);
    if (options[i].size() >= 2 && options[i][0] == '"' && options[i][options[i].size()-1] == '"')
    {
      newSeries._legend = SLR::UnQuote(options[i]);
    }
    else if (ToUpper(options[i]) == "NOLEGEND")
    {
      newSeries.noLegend = true;
    }
    else if (ToUpper(options[i]) == "BOLD")
    {
      newSeries.bold = true;
    }
    else if (ToUpper(options[i]) == "NEGATE")
    {
      newSeries.negate = true;
    }
    else if (ToUpper(options[i]) == "FORCE")
    {
      force = true;
    }
    else if (!HasLetters(options[i]))
    {
      if (colorNum < 3)
      {
        color[colorNum] = (float)atof(options[i].c_str());
        autoColor = false;
        colorNum++;
      }
    }
  }

  
  newSeries._yName = path;
  
  newSeries._objName = SLR::LeftOf(newSeries._yName, '.');
  newSeries._fieldName = newSeries._yName.substr(newSeries._objName.size() + 1);

  // If the series is already plotted, then don't add the series again => return
  if (!force && IsSeriesPlotted(path))
  {
    return;
  }

  newSeries._color = color;
  if (autoColor)
  {
    float hue = ((float)_series.size())*30.f;
    newSeries._color = HSVtoRGB(hue + 15.f , 1, 1);
  }
  _series.push_back(newSeries);
}

bool Graph::IsSeriesPlotted(string path)
{
  // Loop through the series vector and check if the field already exists there
  for (unsigned int i = 0; i < _series.size(); i++)
  {

    if (!SLR::ToUpper(path).compare(SLR::ToUpper(_series.at(i)._yName)))
    {
      return true;
    }
  }

  return false;
}

void Graph::RemoveAllElements()
{
  _title = "";
  _series.clear();
  _analyzers.clear(); 
  _graphYLow = -numeric_limits<float>::infinity();
  _graphYHigh = numeric_limits<float>::infinity();
}

void Graph::Reset()
{
  for (unsigned int i = 0; i < _series.size(); i++)
  {
    _series[i].Clear();
  }

  _graphYLow = -numeric_limits<float>::infinity();
  _graphYHigh = numeric_limits<float>::infinity();
}

void Graph::Clear()
{
  for (unsigned i = 0; i < _analyzers.size(); i++)
  {
    _analyzers[i]->Reset();
  }

  if (_series.empty())
  {
    return;
  }

  for (unsigned int i = 0; i < _series.size(); i++)
  {
    _series[i].Clear();
  }

	// if we were logging, stop logging, and reopen the file
	if (_logFile)
	{
		fclose(_logFile);
		_logFile = NULL;
		BeginLogToFile();
	}
}

void Graph::BeginLogToFile()
{
	if (_logFile != NULL) return;

	string path = "../config/log/" + _name + ".txt";
	_logFile = fopen(path.c_str(), "w");
	
	if (_logFile)
	{
		fprintf(_logFile, "time");
		for (unsigned int i = 0; i < _series.size(); i++)
		{
			fprintf(_logFile, ", ");
			fprintf(_logFile, "%s", _series[i]._yName.c_str());
		}
		fprintf(_logFile, "\n");
		fflush(_logFile);
	}
}

void Graph::Update(double time, std::vector<shared_ptr<DataSource> >& sources)
{
	std::vector<bool> newData(_series.size());
	bool anyNewData = false;

  for (unsigned int i = 0; i < _series.size(); i++)
  {
		newData[i] = false;
    for (unsigned int j = 0; j < sources.size(); j++)
    {
      float tmp;
			if (sources[j]->GetData(_series[i]._yName, tmp))
			{
				newData[i] = true;
				anyNewData = true;
				_series[i].x.push((float)time);
        if (_series[i].negate)
        {
          _series[i].y.push(-tmp);
        }
        else
        {
          _series[i].y.push(tmp);
        }
        break;
      }
    } 
  }

	if (_logFile != NULL && anyNewData)
	{
		fprintf(_logFile, "%f", time);
		for (unsigned int i = 0; i < _series.size(); i++)
		{
			if (newData[i])
			{
				fprintf(_logFile, ",%f", _series[i].y.newest());
			}
			else
			{
				fprintf(_logFile, ",%f", numeric_limits<float>::quiet_NaN());
			}
		}
		fprintf(_logFile, "\n");
		fflush(_logFile);
	}

  for (unsigned i = 0; i < _analyzers.size(); i++)
  {
    _analyzers[i]->Update(time,sources);
  }
}

void GetRange(FixedQueue<float>& f, float& low, float& high)
{
  low = high = 0;
  if (f.n_meas() == 0) return;
  low = high = f[0];
  for (unsigned int i = 1; i < f.n_meas(); i++)
  {
    low = MIN(low, f[i]);
    high = MAX(high, f[i]);
  }

}

void Graph::DrawSeries(Series& s)
{
  if (s.x.n_meas() < 2 || s.x.n_meas() != s.y.n_meas()) return;

  glColor3f(s._color[0], s._color[1], s._color[2]);

  float tmp = 0;
  glGetFloatv(GL_LINE_WIDTH, &tmp);

  if (s.bold)
  {
    glLineWidth(2);    
  }

  glBegin(GL_LINE_STRIP);
  for (unsigned int i = 0; i < s.x.n_meas(); i++)
  {
    glVertex2f(s.x[i], s.y[i]); 
  }
  glEnd();

  glLineWidth(tmp);

}

// Given a data range (r), what is the format string we should use for printing the tick
// labels, how many ticks should there be, and what should the tick labels be?
// This is a rough pragmatic solution that usually produces reasonable results
// 
// Input: low, high -> range of the data of interest
// Returns: string w/ printf format to use for printing the tick labels (e.g. "%.2lf)
// Optional returns:
//    tick: if !null, gets the value at of the 'optimal' tick-tick distance
//	  A: if !null, set to the 'tick' value just below the input range
//	  B: if !null, set to the 'tick' value just above the input range
string GetValueFormat(float low, float high, float* tick, float* A, float* B)
{
  char format[100];

  float range = high - low;
  float T = powf(10.f, floor(log10f(range)));
  if (range / T < 4) T /= 2.f;
  if (range / T > 8) T *= 2.f;

  if (T<1)
  {
    sprintf_s(format, 100, "%%.%dlf", -(int)floor(log10f(T)));
  }
  else
  {
    sprintf_s(format, 100, "%%.0lf");
  }

  if (tick != NULL)		*tick = T;
  if (A != NULL)		*A = low - fmodf(low, T);
  if (B != NULL)		*B = high - fmodf(high, T) + T;
  return format;
}

void Graph::Draw()
{
  if (_series.size() == 0) return;

  // find range
  float lowX = 0, highX = 0, lowY = 0, highY = 0;

  for (unsigned int i = 0; i < _series.size(); i++)
  {
    float tmpLY = lowY, tmpHY = highY;
    GetRange(_series[i].y, tmpLY, tmpHY);
    if (i == 0)
    {
      lowY = tmpLY;
      highY = tmpHY;
    }
    else
    {
      lowY = MIN(lowY, tmpLY);
      highY = MAX(highY, tmpHY);
    }

    float tmpLX = lowX, tmpHX = highX;
    GetRange(_series[i].x, tmpLX, tmpHX);
    if (i == 0)
    {
      lowX = tmpLX;
      highX = tmpHX;
    }
    else
    {
      lowX = MIN(lowX, tmpLX);
      highX = MAX(highX, tmpHX);
    }
  }

  if ((highY - lowY) < 0.001f)
  {
    float mid = (highY + lowY) / 2.f;
    lowY = mid - 0.0005f;
    highY = mid + 0.0005f;
  }

  if ((highX - lowX) < 1.f)
  {
    highX = lowX + 1.f;
  }

  if (_graphYLow != -numeric_limits<float>::infinity())
  {
    lowY = MIN(_graphYLow,lowY);
  }
  if (_graphYHigh != numeric_limits<float>::infinity())
  {
    highY = MAX(_graphYHigh,highY);
  }

  // expand by 10%
  float rangeY = highY - lowY;
  lowY -= rangeY * 0.05f;
  highY += rangeY * 0.05f;
  
  // if we have a title, expand up by further 11%
  if (!_title.empty())
  {
    highY += rangeY * 0.11f;
  }

  lowX -= (highX - lowX) * .1f;

  glPushMatrix();

  glScalef(2.f / (highX - lowX), 2.f / (highY - lowY), 1.f);
  glTranslatef(-(highX + lowX) / 2.f, -(highY + lowY) / 2.f, 0.f);

  
  // y=0 line
  
  glLineWidth(1);
  glBegin(GL_LINES);  
  glColor3f(.5f, .5f, .5f);
  if (0 > lowY && 0 < highY)
  {
    glVertex2f(lowX, 0);
    glVertex2f(highX, 0);
  }
  glEnd();
  

  glLineWidth(1);
  glBegin(GL_LINES);

  // grid
  glColor3f(0.1f, 0.1f, 0.1f);
  
  float tickYDelta = 0, tickYLow = 0, tickYHigh = 0;
  string tickYFormat = GetValueFormat(lowY, highY, &tickYDelta, &tickYLow, &tickYHigh);
  for (float y = tickYLow; y <= tickYHigh; y += tickYDelta)
  {
    if (y < lowY || y> highY || y==0) continue;
    glVertex2f(lowX, y);
    glVertex2f(highX, y);
  }

  glEnd(); // GL_LINES

	for (unsigned int i = 0; i < _series.size(); i++)
	{
		DrawSeries(_series[i]);
	}

  for (unsigned i = 0; i < _analyzers.size(); i++)
  {
    _analyzers[i]->Draw(lowX, highX, lowY, highY);
  }

  // tick labels
  glColor3f(.9f,.9f,.9f);
  for (float y = tickYLow; y <= tickYHigh; y += tickYDelta)
  {
    if (y < (lowY + (highY-lowY)*.05f) || y> (highY - (highY - lowY)*.05f)) continue;
    char buf[100];
    sprintf_s(buf, 100, tickYFormat.c_str(), y);
    DrawStrokeText(buf, lowX + .01f*(highX-lowX), y, 0, 1.2f,(highX-lowX)/3.f, (highY - lowY)/3.f*2.f);
  }
  
  glPopMatrix();
  
  // series names
  int j = 0;
  if (!_title.empty()) j = 1;
  for (unsigned int i = 0; i < _series.size(); i++)
  {
    if (_series[i].noLegend) continue;
    glColor3f(_series[i]._color[0], _series[i]._color[1], _series[i]._color[2]);
    DrawStrokeText_Align(ToLower(_series[i]._legend).c_str(), .95f, .8f - j * .205f, 0, 1.5f, 1.f, 2.f,GLD_ALIGN_RIGHT);
    j++;
  }

  // draw title
  if (!_title.empty())
  {
    glColor3f(1, 1, 1);
    DrawStrokeText_Align(_title.c_str(), 0.01f, .8f, 0, 1.5f, 1.f, 2.f,GLD_ALIGN_CENTER);
  }
}
