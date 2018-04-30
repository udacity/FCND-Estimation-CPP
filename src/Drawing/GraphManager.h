#pragma once

#include "Graph.h"
#include <map>
#include <set>

class DataSource;

class GraphManager
{
public:
  GraphManager(bool own_window=true);
  ~GraphManager();
  void Reset();
  void Clear();
  void UpdateData(double time);
  void DrawUpdate();
  
  void GraphCommand(string path);
  void InitPaint();
  void Paint();

  void RegisterDataSource(shared_ptr<DataSource> src);
  template<typename T>
  void RegisterDataSources(vector<shared_ptr<T> > srcs)
  {
    for (auto i = srcs.begin(); i != srcs.end(); i++)
    {
      RegisterDataSource(*i);
    }
  }
  
  shared_ptr<Graph> graph1, graph2;
  std::vector<shared_ptr<DataSource> > _sources;

  std::vector<std::string> GetGraphableStrings();

protected:
  int _glutWindowNum;
  bool _ownWindow;
};
