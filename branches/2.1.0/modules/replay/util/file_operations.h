/*
* Copyright (c) 2011 Sveriges Television AB <info@casparcg.com>
* Copyright (c) 2013 Technical University of Lodz Multimedia Centre <office@cm.p.lodz.pl>
*
* This file is part of CasparCG (www.casparcg.com).
*
* CasparCG is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* CasparCG is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with CasparCG. If not, see <http://www.gnu.org/licenses/>.
*
* Author: Jan Starzak, jan@ministryofgoodsteps.com
*/

#include <stdint.h>
#include <boost/shared_ptr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "frame_operations.h"

#include <Windows.h>

#pragma once

typedef HANDLE						mjpeg_file_handle;

namespace caspar { namespace replay {

	class mjpeg_file_header {

	public:
		char							magick[4]; // = 'OMAV'
		uint8_t							version; // = 1 for version 1
		size_t							width;
		size_t							height;
		double							fps;
		caspar::core::field_mode		field_mode;
		boost::posix_time::ptime		begin_timecode;

		mjpeg_file_header()
			: field_mode(caspar::core::field_mode::empty)
		{
			
		}
	};

	mjpeg_file_handle safe_fopen(const wchar_t* filename, DWORD mode, DWORD shareFlags);
	void safe_fclose(mjpeg_file_handle file_handle);
	void write_index_header(mjpeg_file_handle outfile_idx, const core::video_format_desc* format_desc);
	void write_index(mjpeg_file_handle outfile_idx, long long offset);
	long long write_frame(mjpeg_file_handle outfile, size_t width, size_t height, mmx_uint8_t* image, short quality);
	long long read_index(mjpeg_file_handle infile_idx);
	long long tell_index(mjpeg_file_handle infile_idx);
	int seek_index(mjpeg_file_handle infile_idx, long long frame, DWORD origin);
	long long tell_frame(mjpeg_file_handle infile);
	int read_index_header(mjpeg_file_handle infile_idx, mjpeg_file_header** header);
	size_t read_frame(mjpeg_file_handle infile, size_t* width, size_t* height, mmx_uint8_t** image);
	int seek_frame(mjpeg_file_handle infile, long long offset, DWORD origin);
}}