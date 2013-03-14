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

#include "replay_consumer.h"

#include <asmlib.h>

#include <common/executor.h>
#include <common/except.h>
#include <common/env.h>
#include <common/log.h>
#include <common/utf.h>
#include <common/future.h>
#include <common/diagnostics/graph.h>

#include <boost/timer.hpp>

#include <core/consumer/frame_consumer.h>
#include <core/frame/frame.h>
#include <core/video_format.h>

#include <boost/thread.hpp>
#include <boost/thread/locks.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <tbb/concurrent_queue.h>

#include "../util/frame_operations.h"
#include "../util/file_operations.h"

#include <setjmp.h>
#include <vector>

#include <Windows.h>

namespace caspar { namespace replay {

struct option
{
	std::string name;
	std::string value;

	option(std::string name, std::string value)
		: name(std::move(name))
		, value(std::move(value))
	{
	}
};
	
struct replay_consumer : boost::noncopyable
{
	core::video_format_desc					format_desc_;
	std::wstring							filename_;
	tbb::atomic<uint64_t>					framenum_;
	int16_t									quality_;
	boost::timer							tick_timer_;
	//boost::mutex*							file_mutex_;
	mjpeg_file_handle						output_file_;
	mjpeg_file_handle						output_index_file_;
	mjpeg_process_mode						mode_;
	bool									file_open_;
	executor								encode_executor_;
	const spl::shared_ptr<diagnostics::graph>		graph_;
	monitor::basic_subject					event_subject_;
	boost::posix_time::ptime				start_timecode_;

#define REPLAY_FRAME_BUFFER					32
#define REPLAY_JPEG_QUALITY					95

public:

	// frame_consumer

	replay_consumer(const std::wstring& filename, const core::video_format_desc& format_desc)
		: filename_(filename)
		, encode_executor_(print())
		, format_desc_(format_desc)
	{
		framenum_ = 0;
		quality_ = REPLAY_JPEG_QUALITY;

		encode_executor_.set_capacity(REPLAY_FRAME_BUFFER);

		graph_->set_color("tick-time", diagnostics::color(0.0f, 0.6f, 0.9f));
		graph_->set_color("frame-time", diagnostics::color(0.1f, 1.0f, 0.1f));
		graph_->set_color("field1-time", diagnostics::color(0.1f, 0.1f, 0.5f));
		graph_->set_color("field2-time", diagnostics::color(0.5f, 0.1f, 0.1f));
		graph_->set_color("dropped-frame", diagnostics::color(0.3f, 0.6f, 0.3f));
		graph_->set_text(print());
		diagnostics::register_graph(graph_);

		//file_mutex_ = new boost::mutex();
		file_open_ = false;

		if (format_desc.field_mode == caspar::core::field_mode::progressive)
		{
			mode_ = PROGRESSIVE;
		}
		else if (format_desc.field_mode == caspar::core::field_mode::upper)
		{
			mode_ = UPPER;
		}
		else if (format_desc.field_mode == caspar::core::field_mode::lower)
		{
			mode_ = LOWER;
		}


		output_file_ = safe_fopen((env::media_folder() + filename_ + L".MAV").c_str(), GENERIC_WRITE, FILE_SHARE_READ);
		if (output_file_ == NULL)
		{
			CASPAR_LOG(error) << print() <<  L" Can't open file " << filename_ << L".MAV for writing";
			return;
		}
		else
		{
			output_index_file_ = safe_fopen((env::media_folder() + filename_ + L".IDX").c_str(), GENERIC_WRITE, FILE_SHARE_READ);
			if (output_index_file_ == NULL)
			{
				CASPAR_LOG(error) << print() <<  L" Can't open index file " << filename_ << L".IDX for writing";
				safe_fclose(output_file_);
				return;
			}
			else
			{
				file_open_ = true;
			}
		}

		start_timecode_ = boost::posix_time::microsec_clock::universal_time();

		write_index_header(output_index_file_, &format_desc, start_timecode_);
		update_osc();
	}

#pragma warning(disable: 4701)
	void encode_video_frame(core::const_frame frame)
	{
		auto format_desc = format_desc_;
		auto out_file = output_file_;
		auto idx_file = output_index_file_;
		auto quality = quality_;

		long long written = 0;
		double perf_timer;

		switch (mode_)
		{
		case PROGRESSIVE:
			written = write_frame(out_file, format_desc.width, format_desc.height, frame.image_data().begin(), quality, PROGRESSIVE, &perf_timer);
			write_index(idx_file, written, NULL);
			break;
		case UPPER:
			written = write_frame(out_file, format_desc.width, format_desc.height / 2, frame.image_data().begin(), quality, UPPER, &perf_timer);
			graph_->set_value("field1-time", perf_timer*0.5*format_desc_.fps);
			write_index(idx_file, written, NULL);
			written = write_frame(out_file, format_desc.width, format_desc.height / 2, frame.image_data().begin(), quality, LOWER, &perf_timer);
			graph_->set_value("field2-time", perf_timer*0.5*format_desc_.fps);
			write_index(idx_file, written, NULL);
			break;
		case LOWER:
			written = write_frame(out_file, format_desc.width, format_desc.height / 2, frame.image_data().begin(), quality, LOWER, &perf_timer);
			graph_->set_value("field1-time", perf_timer*0.5*format_desc_.fps);
			write_index(idx_file, written, NULL);
			written = write_frame(out_file, format_desc.width, format_desc.height / 2, frame.image_data().begin(), quality, UPPER, &perf_timer);
			graph_->set_value("field2-time", perf_timer*0.5*format_desc_.fps);
			write_index(idx_file, written, NULL);
			break;
		}

		++framenum_;
		update_osc();
	}
#pragma warning(default: 4701)

	bool ready_for_frame()
	{
		return encode_executor_.size() < encode_executor_.capacity();
	}

	void update_osc()
	{
		event_subject_ << monitor::event("frame") % (long long)framenum_
					   << monitor::event("time") % (long long)(framenum_ / format_desc_.fps)
					   << monitor::event("path") % filename_
					   << monitor::event("type") % std::wstring(L"replay");
	}

	void mark_dropped()
	{
		graph_->set_tag("dropped-frame");
	}

	boost::unique_future<bool> send(core::const_frame frame)
	{				
		if (file_open_)
		{
			if (ready_for_frame())
			{
				encode_executor_.begin_invoke([=]
				{		
					boost::timer frame_timer;

					encode_video_frame(frame);
			
					graph_->set_text(print());
					graph_->set_value("frame-time", frame_timer.elapsed()*0.5*format_desc_.fps);
				});
			}
			else
			{
				mark_dropped();
			}

			graph_->set_value("tick-time", tick_timer_.elapsed()*format_desc_.fps*0.5);	
			tick_timer_.restart();
		}

		return wrap_as_future(true);
	}

	std::wstring print() const
	{
		return L"replay_consumer[" + filename_ + L".mav|" + std::to_wstring(framenum_) + L"]";
	}

	boost::property_tree::wptree info() const
	{
		boost::property_tree::wptree info;
		info.add(L"type", L"replay-consumer");
		info.add(L"recording-head", framenum_);
		return info;
	}

	void subscribe(const monitor::observable::observer_ptr& o)
	{
		return event_subject_.subscribe(o);
	}

	void unsubscribe(const monitor::observable::observer_ptr& o)
	{
		return event_subject_.unsubscribe(o);
	}	

	~replay_consumer()
	{
		encode_executor_.wait();

		if (output_file_ != NULL)
		{
			safe_fclose(output_file_);
		}

		if (output_index_file_ != NULL)
		{
			safe_fclose(output_index_file_);
		}

		CASPAR_LOG(info) << L"replay_consumer Successfully Uninitialized.";	
	}
};

struct replay_consumer_proxy : public core::frame_consumer
{
	const std::wstring				filename_;
	const std::vector<option>		options_;

	std::unique_ptr<replay_consumer> consumer_;

public:

	replay_consumer_proxy(const std::wstring& filename)
		: filename_(filename)
	{
	}
	
	virtual void initialize(const core::video_format_desc& format_desc, int)
	{
		if(consumer_)
			BOOST_THROW_EXCEPTION(invalid_operation() << msg_info("Cannot reinitialize replay-consumer."));

		consumer_.reset(new replay_consumer(filename_, format_desc));
	}
	
	boost::unique_future<bool> send(core::const_frame frame) override
	{
		return consumer_->send(frame);
	}
	
	std::wstring print() const override
	{
		return consumer_ ? consumer_->print() : L"[replay_consumer]";
	}

	std::wstring name() const override
	{
		return L"replay";
	}

	boost::property_tree::wptree info() const override
	{
		return consumer_ ? consumer_->info() : *(new boost::property_tree::wptree());
	}
		
	bool has_synchronization_clock() const override
	{
		return false;
	}

	int buffer_depth() const override
	{
		return 1;
	}

	int index() const override
	{
		return 150;
	}

	void subscribe(const monitor::observable::observer_ptr& o) override
	{
		consumer_->subscribe(o);
	}

	void unsubscribe(const monitor::observable::observer_ptr& o) override
	{
		consumer_->unsubscribe(o);
	}		
};	

spl::shared_ptr<core::frame_consumer> create_consumer(const std::vector<std::wstring>& params)
{
	if(params.size() < 1 || params[0] != L"REPLAY")
		return core::frame_consumer::empty();

	std::wstring filename = L"REPLAY";

	if (params.size() > 1)
		filename = params[1];

	return spl::make_shared<replay_consumer_proxy>(filename);
}

}}
