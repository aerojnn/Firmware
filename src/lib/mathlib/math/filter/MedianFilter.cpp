/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "MedianFilter.hpp"

namespace math
{

MedianFilter::MedianFilter(int filter_size)
{
    	_filter_size = filter_size;
    	_current_filter_size = 0;
    	_buffer = new float[_filter_size];
}

static int compare(const void* a, const void* b)
{
  	return (*(const float *)a - *(const float *)b );
}

float MedianFilter::apply(float sample)
{
    	// feed buffer
    	if (_current_filter_size < _filter_size) {
        	_buffer[_current_filter_size] = sample;
        	_current_filter_size++;
    	}
    	else {
        	// circular rotation
        	for (int i = 0; i < _filter_size-1; i++) {
            		_buffer[i] = _buffer[i+1];
        	}

        	_buffer[_filter_size-1] = sample;
    	}

    	float _sorted_buffer[_current_filter_size];

    	// copy buffer
    	for (int i = 0; i < _current_filter_size; i++) {
        	_sorted_buffer[i] = _buffer[i];
    	}

    	//std::copy(_buffer   , _buffer +  _current_filter_size, _sorted_buffer);
	qsort(_sorted_buffer, _current_filter_size, sizeof(float), compare);

    	return _sorted_buffer[_current_filter_size/2];
}

}// namespace math
