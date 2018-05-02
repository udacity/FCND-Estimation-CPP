#pragma once

#include <vector>

namespace SLR
{
// helper function -- trimming
inline std::string Trim(const std::string& str,
                 const std::string& whitespace = " \t\n")
{
    const auto strBegin = str.find_first_not_of(whitespace);
    if (strBegin == std::string::npos)
        return ""; // no content

    const auto strEnd = str.find_last_not_of(whitespace);
    const auto strRange = strEnd - strBegin + 1;

    return str.substr(strBegin, strRange);
}

inline std::string ToUpper(const std::string& in)
{
  std::string ret=in;
  for(std::size_t i=0;i<in.size();i++)
  {
    if(ret[i]>='a' && ret[i]<='z') ret[i] -= ('a'-'A');
  }
  return ret;
}

inline std::string ToLower(const std::string& in)
{
  std::string ret = in;
  for (std::size_t i = 0; i<in.size(); i++)
  {
    if (ret[i] >= 'A' && ret[i] <= 'Z') ret[i] += ('a' - 'A');
  }
  return ret;
}


inline std::string CapitalizeFirstLetter(const std::string& in)
{
  std::string ret=in;
  if(in.size()>0)
  { 
    if(ret[0]>='a' && ret[0]<='z') ret[0] -= ('a'-'A');
  }  
  return ret;
}

inline bool Contains(const std::string& s, char c)
{
  const auto i = s.find_first_of(c);
  return i != std::string::npos;
}

inline std::string LeftOf(const std::string& s, char c)
{
  const auto i = s.find_first_of(c);
  if (i == std::string::npos) return s;
  return s.substr(0, i);
}

inline std::string RightOf(const std::string& s, char c)
{
  const auto i = s.find_first_of(c);
  if (i == std::string::npos) return "";
  return s.substr(i + 1);
}

inline std::string RightOfLast(const std::string& s, char c)
{
  const auto i = s.find_last_of(c);
  if (i == std::string::npos) return "";
  return s.substr(i + 1);
}

inline std::string UnQuote(const std::string& s)
{
  string arg = SLR::Trim(s);
  if (arg.size() >= 2 && arg[0] == '"' && arg[arg.size() - 1] == '"')
  {
    arg = arg.substr(1, arg.size() - 2);
  }
  return arg;
}

inline std::vector<string> SimpleFunctionParser(string cmd)
{
  std::vector<string> ret;

  if (cmd.empty()) return ret;
  if (cmd[cmd.size() - 1] != ')') return ret;

  string f = SLR::LeftOf(cmd, '(');
  if (f.find_first_of('"') != string::npos) return ret;

  string args = cmd.substr(f.size() + 1, cmd.size() - f.size() - 2);

  ret.push_back(f);

  bool quote = false;
  unsigned int i = 0, s = 0;
  for (i = 0; i < args.size(); i++)
  {
    if (args[i] == '"')
    {
      quote = !quote;
      if (quote) s = i;
      continue;
    }
    if (args[i] == ',' && !quote)
    {
      ret.push_back(SLR::Trim(args.substr(s, i - s)));
      s = i + 1;
    }
  }

  if (s != i)
  {
    ret.push_back(SLR::Trim(args.substr(s, i - s)));
  }

  return ret;
}

inline std::vector<string> Split(const char* str, char c = ' ')
{
  std::vector<std::string> result;

  do
  {
    const char *begin = str;

    while (*str != c && *str)
    {
      str++;
    }

    result.push_back(std::string(begin, str));
  } while (0 != *str++);

  return result;
}

inline std::vector<std::string> Split(std::string s, char c = ' ')
{
  return Split(s.c_str(), c);
}

inline bool HasLetters(std::string s)
{
	for(std::size_t i = 0; i < s.size(); i++)
	{
		if ((s[i] >= 'a' && s[i] <= 'z') || (s[i] >= 'A' && s[i] <= 'Z'))
		{
			return true;
		}
	}
	return false;
}

} // namespace SLR
