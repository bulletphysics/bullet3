#if defined(__linux__)

/*
 * Copyright (c) 2014 Craig Lilley <cralilley@gmail.com>
 * This software is made available under the terms of the MIT licence.
 * A copy of the licence can be obtained from:
 * http://opensource.org/licenses/MIT
 */

#include <vector>
#include <string>
#include <sstream>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdarg>
#include <cstdlib>

#include <glob.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include "serial/serial.h"

using serial::PortInfo;
using std::cout;
using std::endl;
using std::getline;
using std::ifstream;
using std::istringstream;
using std::string;
using std::vector;

static vector<string> glob(const vector<string>& patterns);
static string basename(const string& path);
static string dirname(const string& path);
static bool path_exists(const string& path);
static string realpath(const string& path);
static string usb_sysfs_friendly_name(const string& sys_usb_path);
static vector<string> get_sysfs_info(const string& device_path);
static string read_line(const string& file);
static string usb_sysfs_hw_string(const string& sysfs_path);
static string format(const char* format, ...);

vector<string>
glob(const vector<string>& patterns)
{
	vector<string> paths_found;

	if (patterns.size() == 0)
		return paths_found;

	glob_t glob_results;

	int glob_retval = glob(patterns[0].c_str(), 0, NULL, &glob_results);

	vector<string>::const_iterator iter = patterns.begin();

	while (++iter != patterns.end())
	{
		glob_retval = glob(iter->c_str(), GLOB_APPEND, NULL, &glob_results);
	}

	for (int path_index = 0; path_index < glob_results.gl_pathc; path_index++)
	{
		paths_found.push_back(glob_results.gl_pathv[path_index]);
	}

	globfree(&glob_results);

	return paths_found;
}

string
basename(const string& path)
{
	size_t pos = path.rfind("/");

	if (pos == std::string::npos)
		return path;

	return string(path, pos + 1, string::npos);
}

string
dirname(const string& path)
{
	size_t pos = path.rfind("/");

	if (pos == std::string::npos)
		return path;
	else if (pos == 0)
		return "/";

	return string(path, 0, pos);
}

bool path_exists(const string& path)
{
	struct stat sb;

	if (stat(path.c_str(), &sb) == 0)
		return true;

	return false;
}

string
realpath(const string& path)
{
	char* real_path = realpath(path.c_str(), NULL);

	string result;

	if (real_path != NULL)
	{
		result = real_path;

		free(real_path);
	}

	return result;
}

string
usb_sysfs_friendly_name(const string& sys_usb_path)
{
	unsigned int device_number = 0;

	istringstream(read_line(sys_usb_path + "/devnum")) >> device_number;

	string manufacturer = read_line(sys_usb_path + "/manufacturer");

	string product = read_line(sys_usb_path + "/product");

	string serial = read_line(sys_usb_path + "/serial");

	if (manufacturer.empty() && product.empty() && serial.empty())
		return "";

	return format("%s %s %s", manufacturer.c_str(), product.c_str(), serial.c_str());
}

vector<string>
get_sysfs_info(const string& device_path)
{
	string device_name = basename(device_path);

	string friendly_name;

	string hardware_id;

	string sys_device_path = format("/sys/class/tty/%s/device", device_name.c_str());

	if (device_name.compare(0, 6, "ttyUSB") == 0)
	{
		sys_device_path = dirname(dirname(realpath(sys_device_path)));

		if (path_exists(sys_device_path))
		{
			friendly_name = usb_sysfs_friendly_name(sys_device_path);

			hardware_id = usb_sysfs_hw_string(sys_device_path);
		}
	}
	else if (device_name.compare(0, 6, "ttyACM") == 0)
	{
		sys_device_path = dirname(realpath(sys_device_path));

		if (path_exists(sys_device_path))
		{
			friendly_name = usb_sysfs_friendly_name(sys_device_path);

			hardware_id = usb_sysfs_hw_string(sys_device_path);
		}
	}
	else
	{
		// Try to read ID string of PCI device

		string sys_id_path = sys_device_path + "/id";

		if (path_exists(sys_id_path))
			hardware_id = read_line(sys_id_path);
	}

	if (friendly_name.empty())
		friendly_name = device_name;

	if (hardware_id.empty())
		hardware_id = "n/a";

	vector<string> result;
	result.push_back(friendly_name);
	result.push_back(hardware_id);

	return result;
}

string
read_line(const string& file)
{
	ifstream ifs(file.c_str(), ifstream::in);

	string line;

	if (ifs)
	{
		getline(ifs, line);
	}

	return line;
}

string
format(const char* format, ...)
{
	va_list ap;

	size_t buffer_size_bytes = 256;

	string result;

	char* buffer = (char*)malloc(buffer_size_bytes);

	if (buffer == NULL)
		return result;

	bool done = false;

	unsigned int loop_count = 0;

	while (!done)
	{
		va_start(ap, format);

		int return_value = vsnprintf(buffer, buffer_size_bytes, format, ap);

		if (return_value < 0)
		{
			done = true;
		}
		else if (return_value >= buffer_size_bytes)
		{
			// Realloc and try again.

			buffer_size_bytes = return_value + 1;

			char* new_buffer_ptr = (char*)realloc(buffer, buffer_size_bytes);

			if (new_buffer_ptr == NULL)
			{
				done = true;
			}
			else
			{
				buffer = new_buffer_ptr;
			}
		}
		else
		{
			result = buffer;
			done = true;
		}

		va_end(ap);

		if (++loop_count > 5)
			done = true;
	}

	free(buffer);

	return result;
}

string
usb_sysfs_hw_string(const string& sysfs_path)
{
	string serial_number = read_line(sysfs_path + "/serial");

	if (serial_number.length() > 0)
	{
		serial_number = format("SNR=%s", serial_number.c_str());
	}

	string vid = read_line(sysfs_path + "/idVendor");

	string pid = read_line(sysfs_path + "/idProduct");

	return format("USB VID:PID=%s:%s %s", vid.c_str(), pid.c_str(), serial_number.c_str());
}

vector<PortInfo>
serial::list_ports()
{
	vector<PortInfo> results;

	vector<string> search_globs;
	search_globs.push_back("/dev/ttyACM*");
	search_globs.push_back("/dev/ttyS*");
	search_globs.push_back("/dev/ttyUSB*");
	search_globs.push_back("/dev/tty.*");
	search_globs.push_back("/dev/cu.*");

	vector<string> devices_found = glob(search_globs);

	vector<string>::iterator iter = devices_found.begin();

	while (iter != devices_found.end())
	{
		string device = *iter++;

		vector<string> sysfs_info = get_sysfs_info(device);

		string friendly_name = sysfs_info[0];

		string hardware_id = sysfs_info[1];

		PortInfo device_entry;
		device_entry.port = device;
		device_entry.description = friendly_name;
		device_entry.hardware_id = hardware_id;

		results.push_back(device_entry);
	}

	return results;
}

#endif  // defined(__linux__)
