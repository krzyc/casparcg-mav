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

#pragma once

namespace caspar { namespace replay {

	struct mjpeg_file_header {
		char							magick[4]; // = 'OMAV'
		uint8_t							version; // = 1 for version 1
		size_t							width;
		size_t							height;
		double							fps;
		caspar::core::field_mode::type	field_mode;
		boost::posix_time::ptime		begin_timecode;
	};

	enum mjpeg_process_mode
	{
		PROGRESSIVE,
		UPPER,
		LOWER
	};

	boost::shared_ptr<FILE> safe_fopen(const char* filename, const char* mode, int shareFlags);
	void write_index_header(boost::shared_ptr<FILE> outfile_idx, const core::video_format_desc* format_desc, boost::posix_time::ptime start_timecode);
	void write_index(boost::shared_ptr<FILE> outfile_idx, long long offset);
	long long write_frame(boost::shared_ptr<FILE> outfile, size_t width, size_t height, const mmx_uint8_t* image, short quality, mjpeg_process_mode mode);
	long long read_index(boost::shared_ptr<FILE> infile_idx);
	long long tell_index(boost::shared_ptr<FILE> infile_idx);
	int seek_index(boost::shared_ptr<FILE> infile_idx, long long frame, int origin);
	long long tell_index(boost::shared_ptr<FILE> infile_idx);
	int read_index_header(boost::shared_ptr<FILE> infile_idx, mjpeg_file_header** header);
	size_t read_frame(boost::shared_ptr<FILE> infile, size_t* width, size_t* height, mmx_uint8_t** image);
	int seek_frame(boost::shared_ptr<FILE> infile, long long offset, int origin);
}}