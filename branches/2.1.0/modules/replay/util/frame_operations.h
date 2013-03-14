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

#pragma once

namespace caspar { namespace replay {
void bgra_to_rgb(const uint8_t* src, uint8_t* dst, int line_width);
void rgb_to_bgra(const uint8_t* src, uint8_t* dst, int line_width);
void split_frame_to_fields(const uint8_t* src, uint8_t* dst1, uint8_t* dst2, size_t width, size_t height, size_t stride);
void interlace_fields(const uint8_t* src1, const uint8_t* src2, uint8_t* dst, size_t width, size_t height, size_t stride);
void field_double(const uint8_t* src, uint8_t* dst, size_t width, size_t height, size_t stride);
void blend_images(const uint8_t* src1, uint8_t* src2, uint8_t* dst, size_t width, size_t height, size_t stride, uint8_t level);
}}