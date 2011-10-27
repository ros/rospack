/*
 * Copyright (C) 2008, Willow Garage, Inc., Morgan Quigley
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

#include "rospack/rospack_backcompat.h"
#include "rospack/rospack.h"

#include <boost/algorithm/string.hpp>
#include <string.h>
#include <stdio.h>
#include <errno.h>

namespace rospack
{

int 
ROSPack::run(int argc, char** argv)
{
  std::string cmd;
  for(int i=0; i<argc; i++)
  {
    if(i==0)
      cmd.append(argv[i]);
    else
      cmd.append(std::string(" ") + argv[i]);
  }
  return run(cmd);
}

int 
ROSPack::run(const std::string& cmd)
{
  // Callers of this method don't make 'rospack' argv[0].
  std::string full_cmd = std::string("rospack ") + cmd;

  FILE* p;
  if(!(p = popen(full_cmd.c_str(), "r")))
  {
    std::string errmsg = 
            std::string("failed to execute rospack command ") + full_cmd + 
            ": " + strerror(errno);
    fprintf(stderr, "[rospack] Error: %s\n", errmsg.c_str());
    return 1;
  }
  else
  {
    char buf[8192];
    memset(buf,0,sizeof(buf));
    // Read the command's output
    do
    {
      clearerr(p);
      while(fgets(buf + strlen(buf),sizeof(buf)-strlen(buf)-1,p));
    } while(ferror(p) && errno == EINTR);
    // Close the subprocess, checking exit status
    if(pclose(p) != 0)
    {
      std::string errmsg = 
              std::string("got non-zero exit status from executing rospack command ") +
              cmd;
      return 1;
    }
    else
    {
      output_ = buf;
    }
  }
  return 0;
}

} // namespace rospack
