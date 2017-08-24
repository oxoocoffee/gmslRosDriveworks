#ifndef PATH_UTIL_H
#define PATH_UTIL_H

// Author Robert J. Gebis

#include <string>
#include <algorithm>
#include "buildType.h"

using namespace std;

class PathUtil
{
    public:
        static string getPath(const string& filePath)
        {
#if DEBUG
            if(filePath.empty())
                throw std::runtime_error("Empty/Invalid file name");
#endif

            if( filePath.empty() )
                return filePath;

            string::size_type pos = filePath.find_last_of(PATH_SEPARATOR);

            if( pos == string::npos )
                return "";

            pos++;

            return filePath.substr(0, pos);
        }

        static string getFile(const string& filePath)
        {
#if DEBUG
            if(filePath.empty())
                throw std::runtime_error("Empty/Invalid file name");
#endif

            if( filePath.empty() )
                return filePath;

            string::size_type pos = filePath.find_last_of(PATH_SEPARATOR);

            if( pos == string::npos )
                return filePath;

            pos++;

            return filePath.substr(pos, filePath.length() - pos );
        }

        static string getFileName(const string& filePath)
        {
            string str = PathUtil::getFile(filePath);

            if( str.length() )
            {
                string::size_type idx = str.find_last_of('.');

                if( idx == string::npos )
                    return str;

                return str.substr(0, idx);
            }
            else return str;
        }

        static string getFileExt(const string& filePath)
        {
            string str = PathUtil::getFile(filePath);

            if( str.length() )
            {
                string::size_type idx = str.find_last_of('.');

                if( idx == string::npos )
                    return str;

                idx++;

                if(str.length() > idx)
                    str = str.substr(idx);
                else
                    str.clear();
            }

            return str;
        }

        static string replaceFile(const string& filePath, const string& newFile)
        {
            return getPath( forceSystemPath(filePath) ) +
                   getFile( forceSystemPath(newFile) );
        }

        static string getSystemPath(const string& filePath)
        {
            if( filePath.empty() )
                return filePath;

            string temp = filePath;

            if( PATH_SEPARATOR == '/' )
                std::replace(temp.begin(), temp.end(), '\\', PATH_SEPARATOR);
            else
                std::replace(temp.begin(), temp.end(), '/', PATH_SEPARATOR);

            if(temp[temp.length()-1] != PATH_SEPARATOR)
                temp.push_back(PATH_SEPARATOR);

            return temp;
        }

        static string forceSystemPath(const string& filePath)
        {
#if DEBUG
            if(filePath.empty())
                throw std::runtime_error("Empty/Invalid file name");
#endif

            if( filePath.empty() )
                return filePath;

            string temp = filePath;

            if( PATH_SEPARATOR == '/' )
                std::replace(temp.begin(), temp.end(), '\\', PATH_SEPARATOR);
            else
                std::replace(temp.begin(), temp.end(), '/', PATH_SEPARATOR);

            return temp;
        }

        static string  fileLocation(const string& filePath,
                                    uint32_t      line)
        {
#if DEBUG
            if(filePath.empty())
                throw std::runtime_error("Empty/Invalid file name");
#endif
            string s = getFile(filePath);

            string info("\n!! Guru Meditation !!! ");

            info += "\n -> Source: " + PathUtil::getFile(filePath);
            info += " @ ";
            info += std::to_string(line);
            info += "\n -> System: ";
            // TODO: Finish fileLocation
            //info += IMAGE_BUILD_SYSTEM;
            ///info += "\n -> Build : ";
            //info += IMAGE_BUILD_VERSION;

            return info;
        }

        static string forceFileExt(const string& filePath, const string& extention)
        {
#if DEBUG
            if(filePath.empty())
                throw std::runtime_error("Empty/Invalid file name");
            if(extention.empty())
                throw std::runtime_error("Empty/Invalid file extantion");
#endif
            string path = getSystemPath( getPath(filePath) );
            string file = getFileName(filePath);

            return path + file + "." + extention;
        }

        static string getDevPath(const string& dev)
        {
#if DEBUG
            if(dev.empty())
                throw std::runtime_error("Empty/Invalid dev name");
#endif
            string path = getSystemPath( getPath(dev) );

            if( path.empty() )
                return string("/dev/") + dev;
            else
                return dev;
        }
};

#if DEBUG
    #define ERROR_LOCATION PathUtil::fileLocation(__FILE__, __LINE__)
#else
    #define ERROR_LOCATION std::string() 
#endif

#endif // PATH_UTIL_H
