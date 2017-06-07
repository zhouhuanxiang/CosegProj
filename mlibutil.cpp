#define STB_IMAGE_IMPLEMENTATION
#include "mlibutil.h"

#include <string>
#include <iostream>
#include <core-base/common.h>
#include <Windows.h>
#include <fileapi.h>
namespace ml
{

	void assertFunctionMLIB(bool statement, const std::string &description)
	{
		if (!statement)
		{
			std::cout << description << std::endl;
		}
	}

	namespace util {
		UINT64 getFileSize(const std::string &filename)
		{
			bool success;
			WIN32_FILE_ATTRIBUTE_DATA fileInfo;
			success = GetFileAttributesExA(filename.c_str(), GetFileExInfoStandard, (void*)&fileInfo);
			MLIB_ASSERT_STR(success != 0, std::string("GetFileAttributesEx failed on ") + filename);
			//return fileInfo.nFileSizeLow + fileInfo.nFileSizeHigh;
			LARGE_INTEGER size;
			size.HighPart = fileInfo.nFileSizeHigh;
			size.LowPart = fileInfo.nFileSizeLow;
			return size.QuadPart;
		}

		bool fileExists(const std::string &filename)
		{
			std::ifstream file(filename);
			return (!file.fail());
		}

		std::istream& safeGetline(std::istream& is, std::string& t)
		{
			t.clear();

			// The characters in the stream are read one-by-one using a std::streambuf.
			// That is faster than reading them one-by-one using the std::istream.
			// Code that uses streambuf this way must be guarded by a sentry object.
			// The sentry object performs various tasks,
			// such as thread synchronization and updating the stream state.

			std::istream::sentry se(is, true);
			std::streambuf* sb = is.rdbuf();

			for (;;) {
				int c = sb->sbumpc();
				switch (c) {
				case '\n':
					return is;
				case '\r':
					if (sb->sgetc() == '\n')
						sb->sbumpc();
					return is;
				case EOF:
					// Also handle the case when the last line has no line ending
					if (t.empty())
						is.setstate(std::ios::eofbit);
					return is;
				default:
					t += (char)c;
				}
			}
		}
		std::string directoryFromPath(const std::string& path)
		{
			if (path.size() == 0) return path;
			if (path.back() == '\\' || path.back() == '/') return path;
			size_t c = path.size();
			while (c) {
				c--;
				if (path[c] == '\\' || path[c] == '/') {
					if (c == 0) return "";
					else return path.substr(0, c) + '/';
				}
			}
			return "";
		}
	};

	void warningFunctionMLIB(const std::string &description)
	{
		std::cout << description << std::endl;
		//DEBUG_BREAK;
	}


}  // namespace ml