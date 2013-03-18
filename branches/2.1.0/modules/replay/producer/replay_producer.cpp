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
* Author: Robert Nagy, ronag89@gmail.com
*		  Jan Starzak, jan@ministryofgoodsteps.com
*/

#define NOMINMAX
#define WIN32_LEAN_AND_MEAN

#include <limits>
#include <Windows.h>

#include "replay_producer.h"

#include <asmlib.h>

#include <core/video_format.h>

#include <common/env.h>
#include <common/log.h>
#include <common/except.h>
#include <common/param.h>
#include <common/diagnostics/graph.h>
#include <common/utf.h>

#include <boost/assign.hpp>
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/thread.hpp>
#include <boost/thread/locks.hpp>
#include <boost/regex.hpp>
#include <boost/timer.hpp>

#include <tbb/concurrent_queue.h>

#include <algorithm>

#include <sys/stat.h>
#include <math.h>

#include <Windows.h>

#include <core/producer/frame_producer.h>
#include <core/frame/frame.h>
#include <core/frame/draw_frame.h>
#include <core/frame/frame_factory.h>
#include <core/frame/pixel_format.h>
#include <core/monitor/monitor.h>

#include "../util/frame_operations.h"
#include "../util/file_operations.h"

using namespace caspar::core;

namespace caspar { namespace replay {

struct replay_producer : public core::frame_producer_base
{	
	const std::wstring						filename_;
	core::draw_frame						frame_;
	mjpeg_file_handle						in_file_;
	mjpeg_file_handle						in_idx_file_;
	spl::shared_ptr<mjpeg_file_header>		index_header_;
	spl::shared_ptr<core::frame_factory>	frame_factory_;
	tbb::atomic<uint64_t>					framenum_;
	tbb::atomic<uint64_t>					first_framenum_;
	tbb::atomic<uint64_t>					last_framenum_;
	tbb::atomic<uint64_t>					result_framenum_;
	uint8_t*								last_field_;
	size_t									last_field_size_;
	float									left_of_last_field_;
	bool									interlaced_;
	float									speed_;
	float									abs_speed_;
	int										frame_divider_;
	int										frame_multiplier_;
	bool									reverse_;
	bool									seeked_;
	const spl::shared_ptr<diagnostics::graph>		graph_;
	monitor::basic_subject					event_subject_;

	explicit replay_producer(const spl::shared_ptr<core::frame_factory>& frame_factory, const std::wstring& filename, const int sign, const long long start_frame, const long long last_frame, const float start_speed) 
		: filename_(filename)
		, frame_(core::draw_frame::empty())
		, frame_factory_(frame_factory)
	{
		result_framenum_ = 0;
		framenum_ = 0;
		last_framenum_ = 0;
		first_framenum_ = 0;
		left_of_last_field_ = 0;

		last_field_ = NULL;

		seeked_ = false;

		in_file_ = safe_fopen(filename_.c_str(), GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE);
		if (in_file_ != NULL)
		{
			uintmax_t size = 0;

			in_idx_file_ = safe_fopen(boost::filesystem::wpath(filename_).replace_extension(L".idx").c_str(), GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE);
			if (in_idx_file_ != NULL)
			{
				while (size == 0)
				{
					size = boost::filesystem::file_size(boost::filesystem::wpath(filename_).replace_extension(L".idx").string());

					if (size > 0) {
						mjpeg_file_header* header;
						read_index_header(in_idx_file_, &header);
						//index_header_ = boost::shared_ptr<mjpeg_file_header>(header);
						index_header_ = spl::make_shared<mjpeg_file_header>(*header);

						set_playback_speed(start_speed);

						CASPAR_LOG(info) << print() << L" File starts at: " << boost::posix_time::to_iso_wstring(index_header_->begin_timecode);

						if (index_header_->field_mode == caspar::core::field_mode::progressive)
						{
							interlaced_ = false;
						}
						else
						{
							interlaced_ = true;
						}

						if (start_frame > 0)
						{
							long long frame_pos;
							if (interlaced_)
								frame_pos = (long long)(start_frame * 2.0);
							else
								frame_pos = (long long)start_frame;
				
							seek(frame_pos, sign);
						}

						if (last_frame > 0)
						{
							last_framenum_ = start_frame + last_frame;
							if (interlaced_)
								last_framenum_ = last_framenum_ * 2;
						}

						update_osc();

						graph_->set_color("frame-time", diagnostics::color(0.1f, 1.0f, 0.1f));
						graph_->set_text(print());
						diagnostics::register_graph(graph_);
					}
					else
					{
						CASPAR_LOG(warning) << print() << L" Waiting for index file to grow.";
						boost::this_thread::sleep(boost::posix_time::milliseconds(10));
					}
				}
			}
			else
			{
				CASPAR_LOG(error) << print() << L" Index file " << boost::filesystem::wpath(filename_).replace_extension(L".idx").wstring() << " not found";
			}
		}
		else
		{
			CASPAR_LOG(error) << print() << L" Video essence file " << filename_ << " not found";
		}
		//frame_factory_ = frame_factory;
	}

	void update_osc()
	{
		 event_subject_ << monitor::event("replay/path") % filename_
						<< monitor::event("replay/speed") % speed_
						<< monitor::event("replay/frame") % (long long)(interlaced_ ? framenum_ / 2 : framenum_)
						<< monitor::event("replay/time") % (long long)(interlaced_ ? (framenum_ / (2 * index_header_->fps)) : (framenum_ / index_header_->fps))
						<< monitor::event("replay/start-timecode") % boost::posix_time::to_iso_wstring(index_header_->begin_timecode)
						<< monitor::event("replay/fps") % index_header_->fps
						<< monitor::event("replay/video/width") % (int)index_header_->width
						<< monitor::event("replay/video/height") % (int)index_header_->height
		//event_subject_ << monitor::event("replay/video/field") % index_header_->field_mode;
						<< monitor::event("replay/video/codec") % std::wstring(L"MJPEG");

		if (last_framenum_ > 0)
		{
			event_subject_ << monitor::event("replay/length") % (long long)(interlaced_ ? ((last_framenum_ - first_framenum_) / 2) : (last_framenum_ - first_framenum_));
		}
	}
	
	core::draw_frame make_frame(uint8_t* frame_data, size_t size, size_t width, size_t height, bool drop_first_line)
	{
		core::pixel_format_desc desc;
		desc.format = core::pixel_format::bgra;
		desc.planes.push_back(core::pixel_format_desc::plane(width, height, 4));
		auto frame = frame_factory_->create_frame(this, desc);
		if (!drop_first_line)
		{
			std::copy_n(frame_data, size, frame.image_data().begin());
			//A_memcpy(frame_data, frame.image_data().begin(), size);
		}
		else
		{
			size_t line = width * 4;
			std::copy_n(frame_data, size - line, frame.image_data().begin() + line);
			//A_memcpy(frame_data, frame.image_data().begin() + line, size - line);
		}
		frame_ = core::draw_frame(std::move(frame));
		return frame_;
	}

	long long length_index()
	{
		uintmax_t size = boost::filesystem::file_size(boost::filesystem::wpath(filename_).replace_extension(L".idx").string());
		long long el_size = (size - sizeof(mjpeg_file_header)) / sizeof(long long);
		return el_size;
	}

	virtual boost::unique_future<std::wstring> call(const std::wstring& param) override
	{
		boost::promise<std::wstring> promise;
		promise.set_value(do_call(param));
		return promise.get_future();
	}

	std::wstring do_call(const std::wstring& param)
	{
		static const boost::wregex speed_exp(L"SPEED\\s+(?<VALUE>[\\d.-]+)", boost::regex::icase);
		static const boost::wregex pause_exp(L"PAUSE", boost::regex::icase);
		static const boost::wregex seek_exp(L"SEEK\\s+(?<SIGN>[\\+\\-\\|])?(?<VALUE>[\\d]+)", boost::regex::icase);
		static const boost::wregex length_exp(L"LENGTH\\s+(?<VALUE>[\\d]+)", boost::regex::icase);
		
		boost::wsmatch what;
		if(boost::regex_match(param, what, pause_exp))
		{
			set_playback_speed(0.0f);
			return L"";
		}
		if(boost::regex_match(param, what, speed_exp))
		{
			if(!what["VALUE"].str().empty())
			{
				float speed = boost::lexical_cast<float>(what["VALUE"].str());
				set_playback_speed(speed);
			}
			return L"";
		}
		if(boost::regex_match(param, what, seek_exp))
		{
			int sign = 0;
			if(!what["SIGN"].str().empty())
			{
				if (what["SIGN"].str() == L"+")
					sign = 1;
				else if (what["SIGN"].str() == L"|")
					sign = -2;
				else if (what["SIGN"].str() == L"-")
					sign = -1;
			}
			if(!what["VALUE"].str().empty())
			{
				double position = boost::lexical_cast<double>(what["VALUE"].str());
				long long frame_pos = 0;
				if (interlaced_)
					frame_pos = (long long)(position * 2.0);
				else
					frame_pos = (long long)position;
				
				seek(frame_pos, sign);
			}
			return L"";
		}
		if(boost::regex_match(param, what, length_exp))
		{
			if(!what["VALUE"].str().empty())
			{
				long long last_frame = boost::lexical_cast<long long>(what["VALUE"].str());
				last_framenum_ = first_framenum_ + last_frame;
				if (interlaced_)
					last_framenum_ = last_framenum_ * 2;

				event_subject_ << monitor::event("replay/length") % (long long)(interlaced_ ? ((last_framenum_ - first_framenum_) / 2) : (last_framenum_ - first_framenum_));
			}
			return L"";
		}

		CASPAR_THROW_EXCEPTION(invalid_argument());
	}

	void seek(long long frame_pos, int sign)
	{
		if (sign == 0)
		{
			framenum_ = frame_pos;
			seek_index(in_idx_file_, frame_pos, FILE_BEGIN);
		}
		else if (sign == -2)
		{
			framenum_ = length_index() - frame_pos - 4;
			seek_index(in_idx_file_, framenum_, FILE_CURRENT);
		}
		else
		{
			framenum_ = framenum_ + (sign * frame_pos);
			seek_index(in_idx_file_, (frame_pos) * sign, FILE_BEGIN);
		}
		first_framenum_ = framenum_;
		seeked_ = true;
	}

	void set_playback_speed(float speed)
	{
		speed_ = speed;
		abs_speed_ = abs(speed);
		if (speed != 0.0f)
			frame_divider_ = abs((int)(1.0f / speed));
		else 
			frame_divider_ = 0;
		frame_multiplier_ = abs((int)(speed));
		reverse_ = (speed >= 0.0f) ? false : true;
	}

	void update_diag(double elapsed)
	{
		graph_->set_text(print());
		graph_->set_value("frame-time", elapsed*0.5);
	}

	void move_to_next_frame()
	{
		if ((reverse_) && (framenum_ > 0))
		{
			framenum_ -= -(frame_multiplier_ > 1 ? frame_multiplier_ : 1);
			seek_index(in_idx_file_, -1 - (frame_multiplier_ > 1 ? frame_multiplier_ : 1), FILE_CURRENT);
		}
		else
		{
			framenum_ += (frame_multiplier_ > 1 ? frame_multiplier_ : 1);
			if (frame_multiplier_ > 1)
			{
				seek_index(in_idx_file_, frame_multiplier_, FILE_CURRENT);
			}
		}
	}

	void sync_to_frame()
	{
		if (index_header_->field_mode != caspar::core::field_mode::progressive)
		{
			if (
			   ((framenum_ % 2 != 0))
			)
			{
				//CASPAR_LOG(warning) << L" Frame number was " << framenum_ << L", syncing to First Field";
				(void) read_index(in_idx_file_);
				framenum_++;
			}
		}
	}

	void proper_interlace(const uint8_t* field1, const uint8_t* field2, uint8_t* dst)
	{
		if (index_header_->field_mode == caspar::core::field_mode::lower)
		{
			interlace_fields(field2, field1, dst, index_header_->width, index_header_->height, 4);
		}
		else
		{
			interlace_fields(field1, field2, dst, index_header_->width, index_header_->height, 4);
		}
	}

	/* float modf(float a)
	{
		int intpart = (int)a;
		float decpart = a - intpart;
		return decpart;
	} */

	core::draw_frame receive_impl()
	{
		boost::timer frame_timer;

		if (last_framenum_ > 0)
		{
			if (last_framenum_ <= framenum_)
			{
				update_diag(frame_timer.elapsed());
				// frame_ = core::draw_frame::eof();
				return frame_;
			}
		}

		// IF is paused
		if (speed_ == 0.0f) 
		{
			if (!seeked_)
			{
				result_framenum_++;
				update_diag(frame_timer.elapsed());
				return frame_;
			}
			else
			{
				seeked_ = false;
			}
		}
		// IF speed is less than 0.5x and it's not time for a new frame
		else if (abs_speed_ < 0.5f)
		{
			if (result_framenum_ % frame_divider_ > 0)
			{
				result_framenum_++;
				update_diag(frame_timer.elapsed());
				return frame_; // Return previous frame
			}
		}
		else if ((abs_speed_ > 0.5f) && (abs_speed_ < 1.0f))
		{
			// There is no last_field_ to base new field on
			if (last_field_ == NULL)
			{
				sync_to_frame();

				//printf("Generating new frame: %f\n", left_of_last_field_);

				long long field1_pos = read_index(in_idx_file_);

				if (field1_pos == -1)
				{	// There are no more frames
					result_framenum_++;
					update_diag(frame_timer.elapsed());
					return frame_;
				}

				move_to_next_frame();

				seek_frame(in_file_, field1_pos, FILE_BEGIN);

				uint8_t* field1 = NULL;
				uint8_t* field2 = NULL;
				uint8_t* full_frame = NULL;
				size_t field1_width;
				size_t field1_height;
				size_t field1_size = read_frame(in_file_, &field1_width, &field1_height, &field1);

				if (interlaced_)
				{
					long long field2_pos = read_index(in_idx_file_);

					move_to_next_frame();

					seek_frame(in_file_, field2_pos, FILE_BEGIN);

					size_t field2_size = read_frame(in_file_, &field1_width, &field1_height, &field2);

					full_frame = new uint8_t[field1_size * 2];
					proper_interlace(field1, field2, full_frame);

					make_frame(full_frame, field1_size * 2, index_header_->width, index_header_->height, false);

					last_field_size_ = field2_size + field1_size;
				}
				else
				{
					full_frame = field1;
					last_field_size_ = field1_size;
					field1 = NULL;
				}

				result_framenum_++;

				last_field_ = full_frame;
				left_of_last_field_ = ((1.0f - (abs_speed_ - 0.5f)) * 2.0f) - 1.0f;

				if (left_of_last_field_ <= 0)
				{
					delete last_field_;
					last_field_ = NULL;
				}

				if (field2 != NULL)
					delete field2;
				if (field1 != NULL)
					delete field1;

				update_diag(frame_timer.elapsed());

				return frame_;
			}
			else
			{
				//printf("Left of last frame: %f\n", left_of_last_field_);
				if (left_of_last_field_ >= 1.0f)
				{
					//printf("Using last frame\n");
					make_frame(last_field_, last_field_size_, index_header_->width, index_header_->height, false);
					left_of_last_field_ -= 1.0f;

					if (left_of_last_field_ <= 0)
					{
						delete last_field_;
						last_field_ = NULL;
					}

					update_diag(frame_timer.elapsed());

					return frame_;
				}

				//printf("Blending with last frame\n");

				long long field1_pos = read_index(in_idx_file_);

				if (field1_pos == -1)
				{	// There are no more frames
					result_framenum_++;
					update_diag(frame_timer.elapsed());
					return frame_;
				}

				move_to_next_frame();

				seek_frame(in_file_, field1_pos, FILE_BEGIN);

				uint8_t* field1 = NULL;
				uint8_t* field2 = NULL;
				uint8_t* full_frame = NULL;
				size_t field1_width;
				size_t field1_height;
				size_t field1_size = read_frame(in_file_, &field1_width, &field1_height, &field1);

				if (interlaced_)
				{
					long long field2_pos = read_index(in_idx_file_);

					move_to_next_frame();

					seek_frame(in_file_, field2_pos, FILE_BEGIN);

				
					size_t field2_size = read_frame(in_file_, &field1_width, &field1_height, &field2);

					full_frame = new uint8_t[field1_size * 2];
					proper_interlace(field1, field2, full_frame);

					last_field_size_ = field2_size + field1_size;
				}
				else
				{
					last_field_size_ = field1_size;
					full_frame = field1;
					field1 = NULL;
				}

				result_framenum_++;

				uint8_t* final_frame = new uint8_t[last_field_size_];

				blend_images(full_frame, last_field_, final_frame, index_header_->width, index_header_->height, 4, (uint8_t)((1.0f - left_of_last_field_) * 64.0f));

				make_frame(final_frame, last_field_size_, index_header_->width, index_header_->height, false);

				delete last_field_;
				last_field_ = full_frame;
				left_of_last_field_ = left_of_last_field_ + ((1.0f - (abs_speed_ - 0.5f)) * 2.0f) - 1.0f;

				if (left_of_last_field_ <= 0)
				{
					delete last_field_;
					last_field_ = NULL;
				}

				delete final_frame;
				if (field2 != NULL)
					delete field2;
				if (field1 != NULL)
					delete field1;

				update_diag(frame_timer.elapsed());

				return frame_;
			}
		}
		else
		{
			if (last_field_ != NULL)
			{
				delete last_field_;
				last_field_ = NULL;
			}
		}


		// ELSE
		if (abs_speed_ >= 1.0f)
			sync_to_frame();

		long long field1_pos = read_index(in_idx_file_);

		if (field1_pos == -1)
		{	// There are no more frames
			result_framenum_++;
			update_diag(frame_timer.elapsed());
			return frame_;
		}

		move_to_next_frame();

		seek_frame(in_file_, field1_pos, FILE_BEGIN);

		uint8_t* field1 = NULL;
		uint8_t* field2 = NULL;
		uint8_t* full_frame = NULL;
		size_t field1_width;
		size_t field1_height;
		size_t field1_size = read_frame(in_file_, &field1_width, &field1_height, &field1);

		if ((!interlaced_) || (frame_divider_ > 1))
		{
			uint8_t* full_frame = NULL;

			if (interlaced_) 
			{
				full_frame = new uint8_t[field1_size * 2];
				field_double(field1, full_frame, index_header_->width, index_header_->height, 4);
			}
			else
			{
				full_frame = field1;
				field1 = NULL;
			}

			result_framenum_++;

			make_frame(full_frame, (interlaced_ ? field1_size * 2 : field1_size), index_header_->width, index_header_->height, (framenum_ % 2 != 1));

			if (field1 != NULL)
				delete field1;

			delete full_frame;

			update_diag(frame_timer.elapsed());

			return frame_;
		}

		if (interlaced_) 
		{
			long long field2_pos = read_index(in_idx_file_);

			move_to_next_frame();

			seek_frame(in_file_, field2_pos, FILE_BEGIN);

			size_t field2_size = read_frame(in_file_, &field1_width, &field1_height, &field2);

			full_frame = new uint8_t[field1_size + field2_size];

			proper_interlace(field1, field2, full_frame);
		
			make_frame(full_frame, field1_size + field2_size, index_header_->width, index_header_->height, false);
		}
		else
		{
			full_frame = field1;
			field1 = NULL;

			make_frame(full_frame, field1_size, index_header_->width, index_header_->height, false);
		}

		if (field1 != NULL)
			delete field1;
		if (field2 != NULL)
			delete field2;
		delete full_frame;

		update_diag(frame_timer.elapsed());

		update_osc();

		return frame_;
	}
		
	core::draw_frame last_frame() override
	{
		return frame_;
	}

#pragma warning (disable: 4244)
	virtual uint32_t nb_frames() const override
	{
		if (last_framenum_ > 0)
		{
			return (uint32_t)((interlaced_ ? ((last_framenum_ - first_framenum_) / 2) : (last_framenum_ - first_framenum_)) / speed_);
		}
		return std::numeric_limits<uint32_t>::max();
	}
#pragma warning (default: 4244)

	std::wstring print() const override
	{
		return L"replay_producer[" + filename_ + L"|" + boost::lexical_cast<std::wstring>(interlaced_ ? framenum_ / 2 : framenum_)
			 + L"|" + boost::lexical_cast<std::wstring>(speed_)
			 + L"]";
	}

	std::wstring name() const override
	{
		return L"replay";
	}

	void subscribe(const monitor::observable::observer_ptr& o) override															
	{
		return event_subject_.subscribe(o);
	}

	void unsubscribe(const monitor::observable::observer_ptr& o) override		
	{
		return event_subject_.unsubscribe(o);
	}

	virtual boost::property_tree::wptree info() const override
	{
		boost::property_tree::wptree info;
		info.add(L"type", L"replay");
		info.add(L"filename", filename_);
		info.add(L"play-head", framenum_);
		info.add(L"speed", speed_);
		return info;
	}

	~replay_producer()
	{
		if (in_file_ != NULL)
		{
			safe_fclose(in_file_);
		}
		if (in_idx_file_ != NULL)
		{
			safe_fclose(in_idx_file_);
		}
	}
};

spl::shared_ptr<core::frame_producer> create_producer(const spl::shared_ptr<core::frame_factory>& frame_factory, const core::video_format_desc& format_desc, const std::vector<std::wstring>& params)
{
	static const std::vector<std::wstring> extensions = boost::assign::list_of(L".mav");
	std::wstring filename = env::media_folder() + L"\\" + params[0];
	
	auto ext = std::find_if(extensions.begin(), extensions.end(), [&](const std::wstring& ex) -> bool
		{					
			return boost::filesystem::is_regular_file(boost::filesystem::wpath(filename).replace_extension(ex));
		});

	if(ext == extensions.end())
		return core::frame_producer::empty();

	int sign = 0;
	long long start_frame = 0;
	long long last_frame = 0;
	float start_speed = 1.0f;

	if (params.size() >= 3) {
		for (uint16_t i=0; i<params.size(); i++) {
			if (params[i] == L"SEEK")
			{
				static const boost::wregex seek_exp(L"(?<SIGN>[\\|])?(?<VALUE>[\\d]+)", boost::regex::icase);
				boost::wsmatch what;
				if(boost::regex_match(params[i+1], what, seek_exp))
				{
				
					if(!what["SIGN"].str().empty())
					{
						if (what["SIGN"].str() == L"|")
							sign = -2;
						else
							sign = 0;
					}
					if(!what["VALUE"].str().empty())
					{
						start_frame = boost::lexical_cast<long long>(u8(what["VALUE"].str()).c_str());
					}
				}
			}
			else if (params[i] == L"SPEED")
			{
				static const boost::wregex speed_exp(L"(?<VALUE>[\\d.-]+)", boost::regex::icase);
				boost::wsmatch what;
				if (boost::regex_match(params[i+1], what, speed_exp))
				{
					if (!what["VALUE"].str().empty())
					{
						start_speed = boost::lexical_cast<float>(what["VALUE"].str());
					}
				}
			}
			else if (params[i] == L"LENGTH")
			{
				static const boost::wregex speed_exp(L"(?<VALUE>[\\d]+)", boost::regex::icase);
				boost::wsmatch what;
				if (boost::regex_match(params[i+1], what, speed_exp))
				{
					if (!what["VALUE"].str().empty())
					{
						last_frame = boost::lexical_cast<long long>(what["VALUE"].str());
					}
				}
			}
		}
	}

	return create_destroy_proxy(spl::make_shared_ptr<replay_producer>(spl::make_shared<replay_producer>(frame_factory, filename + *ext, sign, start_frame, last_frame, start_speed)));
}


}}