/*
 * Copyright (C) 2008, Willow Garage, Inc.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "rospack_cmdline.h"
//#include "../src/rospack.cpp"
#include "utils.h"

#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <algorithm>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>

namespace po = boost::program_options;

int
main(int argc, char** argv)
{
  rospack::Rospack rp;
  std::string name = argv[1];

  std::vector<std::string> search_path;
  if(!rp.getSearchPathFromEnv(search_path))
    return 1;

  // We crawl here because profile (above) does its own special crawl.
  rp.crawl(search_path, true);

  rospack::Stackage* stackage = rp.findWithRecrawl(name); // private
  if(!stackage)
    return 1;
  try
  {
    rp.computeDeps(stackage); // private
    std::vector<rospack::Stackage*> deps_vec;
    rp.gatherDeps(stackage, false, rospack::POSTORDER, deps_vec); // praivate
    std::cout <<"digraph \"" << name << "\" {" << std::endl;
    for(std::vector<rospack::Stackage*>::const_iterator it = deps_vec.begin();
        it != deps_vec.end();
        ++it) {
        for(std::vector<rospack::Stackage*>::const_iterator it2 = (*it)->deps_.begin();
            it2 != (*it)->deps_.end();
            ++it2) {
            std::cout << "\"" << (*it)->name_ << "\" -> \"" << (*it2)->name_ << "\"" << std::endl;
        }
    }
    std::cout <<"}" << std::endl;
  } catch (...) {
  }

  //printf("%s", output.c_str());
  return 0;
}

