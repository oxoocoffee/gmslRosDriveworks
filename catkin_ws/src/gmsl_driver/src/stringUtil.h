#ifndef STRING_UTIL_H
#define STRING_UTIL_H

// Author Robert J. Gebis

#include <string>

using namespace std;

class StringUtil
{
    public:

    static string& padRight(string& str,
                            const size_t num,
                            const char paddingChar = ' ')
    {
        if(num > str.size())
            str.append(num - str.size(), paddingChar);

        return str;
    }

    static string& padLeft(string& str,
                           const size_t num,
                           const char paddingChar = ' ')
    {
        if(num > str.size())
            str.insert(0, num - str.size(), paddingChar);

        return str;
    }

    static TStringVec& split(TStringVec& vec,
                             const string& param,
                             const char delim = ' ')
    {
        std::stringstream ss;
        ss.str(param);
        std::string item;

        vec.clear();

        while (std::getline(ss, item, delim))
            if( ! item.empty() )
                vec.push_back(item);

        return vec;
    }

    static TStringList& split(TStringList& list,
                             const string& param,
                             const char delim = ' ')
    {
        std::stringstream ss;
        ss.str(param);
        std::string item;

        list.clear();

        while (std::getline(ss, item, delim))
            if( ! item.empty() )
                list.push_back(item);

        return list;
    }

    static std::string trimRight(const string& str, char ch = ' ')
    {
        string::size_type Idx = str.find_last_not_of(ch);

        if( Idx != string::npos )
            return str.substr(0, Idx+1);
        else if( str.length() > 0 and str[0] == ch)
            return "";
        else return str;
    }

    static std::string trimLeft(const string& str, char ch = ' ')
    {
        string::size_type Idx = str.find_first_not_of(ch);

        if( Idx != string::npos )
            return str.substr(Idx);
        else if( str.length() > 0 and str[str.length()-1] == ch)
            return "";
        else return str;
    }

    static std::string trimBoth(const string& str, char ch = ' ')
    {
        return trimRight( trimLeft( str, ch), ch);
    }

    static std::string removeAll(const string& str, const string& chars = " ")
    {
        string newstring(str);

        for(string::size_type idx(0); idx < newstring.size(); ++idx)
        {
            newstring.erase(std::remove(newstring.begin(),
                                        newstring.end(),
                                        chars[idx] ),
                            newstring.end());
        }

        return newstring;
    }
};

#endif // STRING_UTIL_H
