/*
* Arden Markin    
* markin@crimson.ua.edu
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
#include "ercf.h"
#include "flight/msg.h"
#include "drivers/spaaro-ercf.h"

void SpaaroERCF::Init(const ERCFConfig &cfg) {
  if (cfg.device != ERCF_NONE) {
    if (!ercf_.Begin()) {
      MsgError("Unable to establish communication with ercf");
    } else {
      installed_ = true;
    }
  } else {
    installed_ = false;
  }
}

void SpaaroERCF::Read(ERCFData * const data) {
  data->installed = installed_;
  if (data->installed) {
    data->new_data = ercf_.Read();
    if (data->new_data) {
      t_healthy_ms_ = 0;
      data->angle = ercf_.angle();
    }
    data->healthy = (t_healthy_ms_ < 10 * UPDATE_PERIOD_MS_);
  }
}
