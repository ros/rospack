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


#ifndef ROSPACK_ROSPACK_H
#define ROSPACK_ROSPACK_H

#include <boost/tr1/unordered_set.hpp>
#include <boost/tr1/unordered_map.hpp>
#include <string>
#include <vector>
#include <list>

#ifdef ROSPACK_API_BACKCOMPAT_V1
  #include "rospack/rospack_backcompat.h"
#endif


namespace rospack
{

typedef enum 
{
  POSTORDER,
  PREORDER
} traversal_order_t;

// Forward declarations
class Stackage;
class DirectoryCrawlRecord;

/**
 * @brief The base class for package/stack ("stackage") crawlers.  Users of the library should
 * use the functionality provided here through one of the derived classes, 
 * Rosstack or Rospack.
 */
class Rosstackage
{
  private:
    std::string manifest_name_;
    std::string cache_name_;
    bool crawled_;
    std::string name_;
    std::string tag_;
    bool quiet_;
    std::vector<std::string> search_paths_;
    std::tr1::unordered_set<std::string> dups_;
    std::tr1::unordered_map<std::string, Stackage*> stackages_;
    void log(const std::string& level, const std::string& msg, bool append_errno);
    void addStackage(const std::string& path);
    void crawlDetail(const std::string& path,
                     bool force,
                     int depth,
                     bool collect_profile_data,
                     std::vector<DirectoryCrawlRecord*>& profile_data,
                     std::tr1::unordered_set<std::string>& profile_hash);
    bool depsOnDetail(const std::string& name, bool direct,
                         std::vector<Stackage*>& deps);
    bool depsDetail(const std::string& name, bool direct,
                    std::vector<Stackage*>& deps);
    bool isStackage(const std::string& path);
    void loadManifest(Stackage* stackage);
    void computeDeps(Stackage* stackage, bool ignore_errors=false);
    void gatherDeps(Stackage* stackage, bool direct, 
                    traversal_order_t order,
                    std::vector<Stackage*>& deps);
    void gatherDepsFull(Stackage* stackage, bool direct, 
                        traversal_order_t order, int depth, 
                        std::tr1::unordered_set<Stackage*>& deps_hash,
                        std::vector<Stackage*>& deps,
                        bool get_indented_deps,
                        std::vector<std::string>& indented_deps);
    std::string getCachePath();
    bool readCache();
    void writeCache();
    bool validateCache();
    bool expandExportString(Stackage* stackage,
                            const std::string& instring,
                            std::string& outstring);
    void depsWhyDetail(Stackage* from,
                       Stackage* to,
                       std::list<std::list<Stackage*> >& acc_list);

  protected:
    /**
     * @brief Constructor, only used by derived classes.
     * @param manifest_name What the manifest is called (e.g., "manifest.xml or stack.xml")
     * @param cache_name What the cache is called (e.g., "rospack_cache" or "rosstack_cache")
     * @param name Name of the tool we're building (e.g., "rospack" or "rosstack")
     * @param tag Name of the attribute we look for in a "depend" tag in a
     *            manifest (e.g., "package" or "stack")
     */
    Rosstackage(const std::string& manifest_name,
                const std::string& cache_name,
                const std::string& name,
                const std::string& tag);

  public:
    /**
     * @brief Destructor.
     */
    virtual ~Rosstackage();

    /**
     * @brief Usage string, to be overridden by derived classes.
     * @return Command-line usage statement.
     */
    virtual const char* usage() { return ""; }
    /**
     * @brief Crawl the filesystem, accumulating a database of
     *        stackages.  May read results from a cache file instead
     *        of crawling.  This method should be called before any 
     *        making any queries (find, list, etc.).
     * @param search_path List of directories to crawl, in precenence
     *                    order.  Directories should be absolute paths.
     * @param force If true, then crawl even if the cache looks valid
     */
    void crawl(const std::vector<std::string>& search_path, bool force);
    /**
     * @brief Is the current working directory a stackage?
     * @param name If in a stackage, then the stackage's name is written here.
     * @return True if the current working directory contains a manifest
     *         file
     */
    bool inStackage(std::string& name);
    /**
     * @brief Control warning and error console output.
     * @param quiet If true, then don't output any warnings or errors to
     *              console.  If false, then output warnings and errors to
     *              stderr (default behavior).
     */
    void setQuiet(bool quiet);
    /**
     * @brief Get the name of the tool that's in use (e.g., "rospack" or "rosstack")
     * @return The name of the tool.
     */
    const std::string& getName() {return name_;}
    /**
     * @brief Helper method to construct a directory search path by looking
     *        at relevant environment variables.  The value of ROS_ROOT goes
     *        first, followed by each element of a colon-separated
     *        ROS_PACKAGE_PATH.
     * @param sp The computed search path is written here.
     * @return True if a search path was computed, false otherwise (e.g., ROS_ROOT not set).
     */
    bool getSearchPathFromEnv(std::vector<std::string>& sp);
    /**
     * @brief Look for a stackage.
     * @param name The stackage to look for.
     * @param path If found, the absolute path to the stackage is written here.
     * @return True if the stackage is found, false otherwise.
     */
    bool find(const std::string& name, std::string& path); 
    /**
     * @brief Compute the packages that are contained in a stack.
     * @param name The stack to work on.
     * @param packages The stack's constituent packages are written here.
     * @return True if the contents could be computed, false otherwise.
     */
    bool contents(const std::string& name, std::vector<std::string>& packages);
    /**
     * @brief Find the stack that contains a package.
     * @param name The package to work on.
     * @param stack If the containing stack is found, its name is written here.
     * @param path If the containing stack is found, its absolute path is written here.
     * @return True if the containing stack could be found, false otherwise.
     */
    bool contains(const std::string& name, 
                  std::string& stack,
                  std::string& path);
    void list(std::vector<std::pair<std::string, std::string> >& list);
    void listDuplicates(std::vector<std::string>& dups);
    bool deps(const std::string& name, bool direct, std::vector<std::string>& deps);
    bool depsOn(const std::string& name, bool direct,
                   std::vector<std::string>& deps);
    bool depsManifests(const std::string& name, bool direct, 
                       std::vector<std::string>& manifests);
    bool depsMsgSrv(const std::string& name, bool direct, 
                    std::vector<std::string>& gens);
    bool depsIndent(const std::string& name, bool direct,
                    std::vector<std::string>& deps);
    bool depsWhy(const std::string& from,
                 const std::string& to,
                 std::string& output);
    bool rosdeps(const std::string& name, bool direct,
                 std::vector<std::string>& rosdeps);
    bool vcs(const std::string& name, bool direct, 
             std::vector<std::string>& vcs);
    bool exports(const std::string& name, const std::string& lang,
                 const std::string& attrib, bool deps_only,
                 std::vector<std::string>& flags);
    bool plugins(const std::string& name, const std::string& attrib, 
                 const std::string& top,
                 std::vector<std::string>& flags);
    bool profile(const std::vector<std::string>& search_path,
                 bool zombie_only,
                 int length,
                 std::vector<std::string>& dirs);
    // Simple console output helpers
    void log_warn(const std::string& msg,
                  bool append_errno = false);
    void log_error(const std::string& msg,
                   bool append_errno = false);
};

class Rospack : public Rosstackage
{
  public:
    Rospack();
    virtual const char* usage();
};

class Rosstack : public Rosstackage
{
  public:
    Rosstack();
    virtual const char* usage();
};

} // namespace rospack

#endif
