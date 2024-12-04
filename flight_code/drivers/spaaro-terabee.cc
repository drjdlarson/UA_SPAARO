/*
* Arden Markin    
* amarkin@crimson.ua.edu
* 
* Copyright (c) 2024 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/


#include "global_defs.h"
#include "hardware_defs.h"
#include "terabee.h"
#include "flight/msg.h"
#include "drivers/spaaro-terabee.h"

void SpaaroTerabee::Init(const TerabeeConfig &cfg) {
  if (cfg.device != TERABEE_NONE) {
    if (!terabee_.Begin()) {
      MsgError("Unable to establish communication with terabee");
    } else {
      installed_ = true;
    }
  } else {
    installed_ = false;
  }
}

void SpaaroTerabee::Read(TerabeeData * const data) {
  data->installed = installed_;
  if (data->installed) {
    // data->new_data = terabee_.Read();
    if (terabee_.Read(&terabee_data_)) {
      t_healthy_ms_ = 0;
      data->new_data = true;
      for (int i=0; i<data->MAX_CH; i++) {
        data->sensor[i].updated = terabee_data_.sensor[i].updated;
        data->sensor[i].range_m = terabee_data_.sensor[i].range_m;
      }

      data->healthy = (t_healthy_ms_ < 10 * UPDATE_PERIOD_MS_);
    }
  }
}
