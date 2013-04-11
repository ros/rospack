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
#include <iostream>
#include <fstream>
#include <map>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>

const char* usage()
{
  return "USAGE: rostree [options] [package]\n\n"
         "  Options:\n"
         "    -h, --help            show this help message and exit\n"
         "    -size                 whether to size by number of depends on 1\n"
         "    -l, --license         color by license type\n"
         "    -q, --quiet           quiet (remove links to common packages)\n"
         "    -o FILENAME           output_filename\n\n"
         "    Example : rostree roscpp | dot | xdot - \n\n";
}

class RosTree {
public:
    RosTree() :
        size(1),
        license(false)
    {
    }
    ~RosTree() {};

    bool rostree_run(std::string pkg_name) {
        std::vector<std::string> search_path;
        if(!rp.getSearchPathFromEnv(search_path))
            return false;
        // We crawl here because profile (above) does its own special crawl.
        rp.crawl(search_path, true);
        rs.crawl(search_path, true);

        std::string path;
        if ( ! rp.find(pkg_name, path)  ) {
            return false;
        }

        process(pkg_name);

        // output
        std::fstream f;
        if ( filename.size() != 0) {
            f.open(filename.c_str(), std::ios::out);
        }
        std::ostream &output = filename.size() == 0 ? std::cout : f;

        typedef std::map<std::string, std::vector<std::string> >::const_reference type;
        output <<"digraph \"" << pkg_name << "\" {" << std::endl;
        int i = 0;
        BOOST_FOREACH(type stack, rostree_stack) {
            output <<"    subgraph cluster" << i++ << " {" << std::endl;
            BOOST_FOREACH(std::string name, stack.second) {
                output <<"      \"" << name << "\"" << std::endl;
            }
            output <<"      label = \"" << stack.first << "\"" << std::endl;
            output <<"    }" << std::endl;
        }
        BOOST_FOREACH(type deps, rostree) {
            BOOST_FOREACH(std::string name, deps.second) {
                output << "    \"" << deps.first << "\" -> \"" << name << "\"" << std::endl;
            }
        }
        output <<"}" << std::endl;

        return true;
    }

    void setLicense(bool l) { license = l; }
    void setSize(int s) { size = s; }
    void setFilename(std::string n) { filename = n; }

    int size;
    bool license;
    std::string filename;

private:
    void process(std::string pkg_name) {
        std::string stack, path;
        bool has_stack = rs.contains(pkg_name, stack, path);
        if ( has_stack ) {
            rostree_stack[stack].push_back(pkg_name);
        }

        std::vector<std::string> deps;
        rp.deps(pkg_name, true, deps);
        BOOST_FOREACH(std::string dep, deps) {
            std::string stack, path;

            if ( std::find(rostree[pkg_name].begin(),
                           rostree[pkg_name].end(),
                           dep ) == rostree[pkg_name].end() ) {
                rostree[pkg_name].push_back(dep);
            }
            process(dep);
        }
    }

    rospack::Rospack rp;
    rospack::Rosstack rs;
    std::map<std::string, std::vector<std::string> > rostree;
    std::map<std::string, std::vector<std::string> > rostree_stack;
};

namespace po = boost::program_options;

bool
parse_args(int argc, char** argv,
           //RosTree& rt, 
           po::variables_map& vm)
{
  po::options_description desc("Allowed options");
  desc.add_options()
          ("package", po::value<std::string>(), "package")
          ("help,h", "help")
          ("size", po::value<int>(),"size")
          ("license,l", "license")
          ("quiet,q", "quiet")
          ("output,o", po::value<std::string>(), "output");

  po::positional_options_description pd;
  pd.add("package", 1);
  try
  {
    po::store(po::command_line_parser(argc, argv).options(desc).positional(pd).run(), vm);
  }
  catch(boost::program_options::error e)
  {
      std::cerr <<  std::string("failed to parse command-line options: ") + e.what() << std::endl;
    return false;
  }
  po::notify(vm);

  return true;
}

int
main(int argc, char** argv)
{
  RosTree rt = RosTree();
  po::variables_map vm;

  if(!parse_args(argc, argv, vm)) {
      return 1;
  }

  if(vm.count("help") || vm.count("package")==0) {
      std::cerr << usage() << std::endl;
      return 0;
  }
  if(vm.count("size")) {
      rt.setSize(vm["output"].as<int>());
  }
  if(vm.count("license")) {
      //rt.setLicense(true);
  }
  if(vm.count("output")) {
      rt.setFilename(vm["output"].as<std::string>());
  }
  if(vm.count("quiet")) {
  }

  std::string pkg_name;
  if(vm.count("package"))
    pkg_name = vm["package"].as<std::string>();

  rt.rostree_run(pkg_name);

  return 0;
}

