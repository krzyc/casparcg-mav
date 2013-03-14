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

#include <boost/shared_ptr.hpp>
#include <stdint.h>
#include <common/env.h>
#include <common/log/log.h>
#include <common/utility/string.h>
#include <common/concurrency/future_util.h>
#include <common/diagnostics/graph.h>

#include <core/consumer/frame_consumer.h>
#include <core/video_format.h>
#include <core/mixer/read_frame.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <jpeglib.h>
#include <setjmp.h>
#include "frame_operations.h"
#include "file_operations.h"

namespace caspar { namespace replay {

	boost::shared_ptr<FILE> safe_fopen(const char* filename, const char* mode, int shareFlags)
	{
		FILE * const fp = _fsopen(filename, mode, shareFlags);
		return fp ? boost::shared_ptr<FILE>(fp, std::fclose) : boost::shared_ptr<FILE>();
	}

	void write_index_header(boost::shared_ptr<FILE> outfile_idx, const core::video_format_desc* format_desc, boost::posix_time::ptime start_timecode)
	{
		mjpeg_file_header	header;
		header.magick[0] = 'O';	// Set the "magick" four bytes
		header.magick[1] = 'M';
		header.magick[2] = 'A';
		header.magick[3] = 'V';
		header.version = 1;	    // Set the version number to 1, for V.1 of the index file format
		header.width = format_desc->width;
		header.height = format_desc->height;
		header.fps = format_desc->fps;
		header.field_mode = format_desc->field_mode;
		header.begin_timecode = start_timecode;

		fwrite(&header, sizeof(mjpeg_file_header), 1, outfile_idx.get());
		fflush(outfile_idx.get());
	}

	int read_index_header(boost::shared_ptr<FILE> infile_idx, mjpeg_file_header** header)
	{
		*header = new mjpeg_file_header();

		size_t read = 0;
		
		read = fread(*header, sizeof(mjpeg_file_header), 1, infile_idx.get());
		if (read != 1)
		{
			delete *header;
			return 1;
		}
		else
		{
			return 0;
		}
	}

	void write_index(boost::shared_ptr<FILE> outfile_idx, long long offset)
	{
		fwrite(&offset, sizeof(long long), 1, outfile_idx.get());
		fflush(outfile_idx.get());
	}

	long long read_index(boost::shared_ptr<FILE> infile_idx)
	{
		long long offset = 0;
		int read = fread(&offset, sizeof(long long), 1, infile_idx.get());
		if (read != 1) offset = -1;
		return offset;
	}

	int seek_index(boost::shared_ptr<FILE> infile_idx, long long frame, int origin)
	{
		switch (origin)
		{
			case SEEK_CUR:
				return _fseeki64_nolock(infile_idx.get(), frame * sizeof(long long), SEEK_CUR);
			case SEEK_SET:
			default:
				return _fseeki64_nolock(infile_idx.get(), frame * sizeof(long long) + sizeof(mjpeg_file_header), SEEK_SET);
		}
	}

	int seek_frame(boost::shared_ptr<FILE> infile, long long offset, int origin)
	{
		return _fseeki64_nolock(infile.get(), offset, origin);
	}

	long long tell_index(boost::shared_ptr<FILE> infile_idx)
	{
		return (_ftelli64_nolock(infile_idx.get()) - sizeof(mjpeg_file_header)) / sizeof(long long);
	}

	struct error_mgr {
		struct jpeg_error_mgr pub;	/* "public" fields */
		jmp_buf setjmp_buffer;	/* for return to caller */
	}; 

	typedef struct error_mgr * error_ptr;

	void error_exit(j_common_ptr cinfo)
	{
		/* cinfo->err really points to a my_error_mgr struct, so coerce pointer */
		error_ptr myerr = (error_ptr) cinfo->err;
		/* Always display the message. */
		/* We could postpone this until after returning, if we chose. */
		(*cinfo->err->output_message) (cinfo);
		/* Return control to the setjmp point */
		longjmp(myerr->setjmp_buffer, 1);
	} 

	size_t read_frame(boost::shared_ptr<FILE> infile, size_t* width, size_t* height, mmx_uint8_t** image)
	{
		struct jpeg_decompress_struct cinfo;

		struct error_mgr jerr;

		JSAMPARRAY buffer;
		int row_stride;
		int rgba_stride;

		cinfo.err = jpeg_std_error(&jerr.pub);
		cinfo.err = jpeg_std_error(&jerr.pub);
		jerr.pub.error_exit = error_exit;
		/* Establish the setjmp return context for my_error_exit to use. */
#pragma warning(disable: 4611)
		if (setjmp(jerr.setjmp_buffer)) {
			/* If we get here, the JPEG code has signaled an error.
			* We need to clean up the JPEG object, close the input file, and return.
			*/
			jpeg_destroy_decompress(&cinfo);
			return 0;
		}
#pragma warning(default: 4611)
		jpeg_create_decompress(&cinfo);

		jpeg_stdio_src(&cinfo, infile.get());

		(void) jpeg_read_header(&cinfo, TRUE); // We ignore the return value - all errors will result in exiting as per setjmp error handler

		(void) jpeg_start_decompress(&cinfo);

		row_stride = cinfo.output_width * cinfo.output_components;
		rgba_stride = cinfo.output_width * 4;

		(*width) = cinfo.output_width;
		(*height) = cinfo.output_height;
		(*image) = new mmx_uint8_t[(*width) * (*height) * 4];

		buffer = (*cinfo.mem->alloc_sarray)((j_common_ptr) &cinfo, JPOOL_IMAGE, row_stride, 1); 

		while (cinfo.output_scanline < cinfo.output_height)
		{
			/* jpeg_read_scanlines expects an array of pointers to scanlines.
			* Here the array is only one element long, but you could ask for
			* more than one scanline at a time if that's more convenient.
			*/
			(void) jpeg_read_scanlines(&cinfo, buffer, 1);
			/* Assume put_scanline_someplace wants a pointer and sample count. */;
			rgb_to_bgra((uint8_t*)(buffer[0]), (uint8_t*)((*image) + ((cinfo.output_scanline - 1) * rgba_stride)), cinfo.output_width);
		} 

		(void) jpeg_finish_decompress(&cinfo);

		jpeg_destroy_decompress(&cinfo);

		return ((*width) * (*height) * 4);
	}

	long long write_frame(boost::shared_ptr<FILE> outfile, size_t width, size_t height, const mmx_uint8_t* image, short quality, mjpeg_process_mode mode)
	{
		long long start_position = _ftelli64(outfile.get());

		// JPEG Compression Parameters
		struct jpeg_compress_struct cinfo;
		// JPEG error info
		struct jpeg_error_mgr jerr;

		JSAMPROW row_pointer[1];
		int row_stride;

		cinfo.err = jpeg_std_error(&jerr);
		jpeg_create_compress(&cinfo);

		jpeg_stdio_dest(&cinfo, outfile.get());

		cinfo.image_width = width;
		cinfo.image_height = height;
		cinfo.input_components = 3;
		cinfo.in_color_space = JCS_RGB;
		cinfo.max_h_samp_factor = 1;
		cinfo.max_v_samp_factor = 1;

		jpeg_set_defaults(&cinfo);

		jpeg_set_quality(&cinfo, quality, TRUE);

		jpeg_start_compress(&cinfo, TRUE);

		// JSAMPLEs per row in image_buffer
		row_stride = width * 4;

		uint8_t* buf = new mmx_uint8_t[row_stride];

		if (mode == PROGRESSIVE) {
			while (cinfo.next_scanline < cinfo.image_height)
			{
				bgra_to_rgb((uint8_t*)(image + (cinfo.next_scanline * row_stride)), buf, width);

				row_pointer[0] = (JSAMPROW)buf;
				(void) jpeg_write_scanlines(&cinfo, row_pointer, 1);
			}
		}
		else if (mode == UPPER)
		{
			while (cinfo.next_scanline < cinfo.image_height)
			{
				bgra_to_rgb((uint8_t*)(image + ((cinfo.next_scanline * 2) * row_stride)), buf, width);

				row_pointer[0] = (JSAMPROW)buf;
				(void) jpeg_write_scanlines(&cinfo, row_pointer, 1);
			}
		}
		else if (mode == LOWER)
		{
			while (cinfo.next_scanline < cinfo.image_height)
			{
				bgra_to_rgb((uint8_t*)(image + ((cinfo.next_scanline * 2 + 1) * row_stride)), buf, width);

				row_pointer[0] = (JSAMPROW)buf;
				(void) jpeg_write_scanlines(&cinfo, row_pointer, 1);
			}
		}
				
		jpeg_finish_compress(&cinfo);
				
		jpeg_destroy_compress(&cinfo); 

		delete buf;

		return start_position;
	}

}}