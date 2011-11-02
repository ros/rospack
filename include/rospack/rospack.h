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

    /**
     * @brief List names and paths of all stackages.
     * @param list Pairs of (name,path) are written here.
     */
    void list(std::vector<std::pair<std::string, std::string> >& list);
    /**
     * @brief Identify duplicate stackages.  Forces crawl.
     * @param dups Names of stackages that are found more than once while
     *             crawling are written here.
     */
    void listDuplicates(std::vector<std::string>& dups);
    /**
     * @brief Compute dependencies of a stackage (i.e., stackages that this
     *        stackages depends on).
     * @param name The stackage to work on.
     * @param direct If true, then compute only direct dependencies.  If
     *               false, then compute full (including indirect)
     *               dependencies.
     * @param deps If dependencies are computed, then they're written here.
     * @return True if dependencies were computed, false otherwise.
     */
    bool deps(const std::string& name, bool direct, std::vector<std::string>& deps);
    /**
     * @brief Compute reverse dependencies of a stackage (i.e., stackages
     *        that depend on this stackage).  Forces crawl.
     * @param name The stackage to work on.
     * @param direct If true, then compute only direct dependencies.  If
     *               false, then compute full (including indirect)
     *               dependencies.
     * @param deps If dependencies are computed, then they're written here.
     * @return True if dependencies were computed, false otherwise.
     */
    bool depsOn(const std::string& name, bool direct,
                   std::vector<std::string>& deps);
    /**
     * @brief List the manifests of a stackage's dependencies.  Used by
     *        rosbuild.
     * @param name The stackage to work on.
     * @param direct If true, then compute only direct dependencies.  If
     *               false, then compute full (including indirect)
     *               dependencies.
     * @param manifests The list of absolute paths to manifests of stackages
     *                  that the given stackage depends on is written here.
     * @return True if the manifest list was computed, false otherwise.
     */
    bool depsManifests(const std::string& name, bool direct, 
                       std::vector<std::string>& manifests);
    /**
     * @brief List the marker files in a packages's dependencies that
     * indicate that those packages contain auto-generated message
     * and/or service code.  Used by rosbuild.
     * @param name The package to work on.
     * @param direct If true, then compute only direct dependencies.  If
     *               false, then compute full (including indirect)
     *               dependencies.
     * @param gens The list of absolute paths to marker files (e.g.,
     * "/opt/ros/electric/stacks/ros_comm/messages/std_msgs/msg_gen/generated")
     * is written here.
     * @return True if the list of files was generated, false otherwise.
     */
    bool depsMsgSrv(const std::string& name, bool direct, 
                    std::vector<std::string>& gens);
    /**
     * @brief Generate indented list of a stackage's dependencies,
     * including duplicates.  Intended for visual debugging of dependency
     * structures.
     * @param name The stackage to work on.
     * @param direct If true, then compute only direct dependencies.  If
     *               false, then compute full (including indirect)
     *               dependencies.
     * @param deps List of the stackage's dependencies, with leading spaces
     * to indicate depth, is written here.  Print this list to console,
     * with newlines separating each element.  Example output:
@verbatim
roscpp_traits
  cpp_common
cpp_common
rostime
  cpp_common
@endverbatim
     * @return True if the indented dependencies were computed, false
     * otherwise.
     */
    bool depsIndent(const std::string& name, bool direct,
                    std::vector<std::string>& deps);
    /**
     * @brief Compute all dependency chains from one stackage to another.
     * Intended for visual debugging of dependency structures.
     * @param from The stackage that depends on.
     * @param to The stackage that is depended on.
     * @param output A list of dependency chains. Print this list to console,
     * with newlines separating each element.  Example output:
@verbatim
Dependency chains from roscpp to roslib:
* roscpp -> roslib 
* roscpp -> std_msgs -> roslib 
* roscpp -> rosgraph_msgs -> std_msgs -> roslib 
@endverbatim
     * @return True if the dependency chains were computed, false
     * otherwise.
     */
    bool depsWhy(const std::string& from,
                 const std::string& to,
                 std::string& output);
    /**
     * @brief Compute rosdep entries that are declared in manifest of a package 
     * and its dependencies.  Used by rosmake.
     * @param name The package to work on.
     * @param direct If true, then compute only direct dependencies.  If
     *               false, then compute full (including indirect)
     *               dependencies.
     * @param rosdeps List of rosdep entries found in the package and its
     * dependencies is written here.
     * @return True if the rosdep list is computed, false otherwise.
     */
    bool rosdeps(const std::string& name, bool direct,
                 std::vector<std::string>& rosdeps);
    /**
     * @brief Compute vcs entries that are declared in manifest of a package 
     * and its dependencies.  Was used by Hudson build scripts; might not
     * be needed.
     * @param name The package to work on.
     * @param direct If true, then compute only direct dependencies.  If
     *               false, then compute full (including indirect)
     *               dependencies.
     * @param vcs List of vcs entries found in the package and its
     * dependencies is written here.
     * @return True if the vcs list is computed, false otherwise.
     */
    bool vcs(const std::string& name, bool direct, 
             std::vector<std::string>& vcs);
    /**
     * @brief Compute exports declared in a package and its dependencies.
     * Used by rosbuild.
     * @param name The package to work on.
     * @param lang The value of the 'lang' attribute to search for.
     * @param attrib The value of the 'attrib' attribute to search for.
     * @param deps_only If true, then only return information from the
     * pacakge's dependencies; if false, then also include the package's
     * own export information.
     * @param flags The accumulated flags are written here.
     * @return True if the flags were computed, false otherwise.
     */
    bool exports(const std::string& name, const std::string& lang,
                 const std::string& attrib, bool deps_only,
                 std::vector<std::string>& flags);
    /**
     * @brief Compute exported plugins declared in packages that depend
     * on a package.  Forces crawl. Used by rosbuild and roslib.
     * @param name The package to work on.
     * @param attrib The value of the 'attrib' attribute to search for.
     * @param top If non-empty, then limit the reverse dependency search to
     * packages that 'top' depends on.  Otherwise, examine all packages
     * that depend on 'name'.
     * @param flags The accumulated flags are written here.
     * @return True if the flags were computed, false otherwise.
     */
    bool plugins(const std::string& name, const std::string& attrib, 
                 const std::string& top,
                 std::vector<std::string>& flags);
    /**
     * @brief Report on time taken to crawl for stackages.  Intended for
     * use in debugging misconfigured stackage trees.  Forces crawl.
     * @param search_path Directories to search; passed to crawl().
     * @param zombie_only If false, then produce formatted output, with
     * timing information.  Example output:
@verbatim
Full tree crawl took 0.014954 seconds.
Directories marked with (*) contain no manifest.  You may
want to delete these directories.
To get just of list of directories without manifests,
re-run the profile with --zombie-only
-------------------------------------------------------------
0.013423   /opt/ros/electric/stacks
0.002989   /opt/ros/electric/stacks/ros_comm
@endverbatim If true, then produce a list of absolute paths that contain no stackages ("zombies"); these directories can likely be safely deleted.  Example output:
@verbatim
/opt/ros/electric/stacks/pr2_controllers/trajectory_msgs
/opt/ros/electric/stacks/pr2_controllers/trajectory_msgs/msg
@endverbatim
     * @param length Limit on how many directories to include in report
     * (ordered in decreasing order of time taken to crawl).
     * @param dirs Profile output. Print this list to console,
     * with newlines separating each element.
     */
    bool profile(const std::vector<std::string>& search_path,
                 bool zombie_only,
                 int length,
                 std::vector<std::string>& dirs);
    /**
     * @brief Log a warning (usually goes to stderr).
     * @param msg The warning.
     * @param append_errno If true, then append a colon, a space, and 
     * the return from 'sterror(errno)'.
     */
    void logWarn(const std::string& msg,
                 bool append_errno = false);
    /**
     * @brief Log a error (usually goes to stderr).
     * @param msg The error.
     * @param append_errno If true, then append a colon, a space, and 
     * the return from 'sterror(errno)'.
     */
    void logError(const std::string& msg,
                  bool append_errno = false);
};

/**
 * @brief Package crawler.  Create one of these to operate on a package
 * tree.  Call public methods inherited from Rosstackage.
 */
class Rospack : public Rosstackage
{
  public:
    /**
     * @brief Constructor
     */
    Rospack();
    /**
     * @brief Usage statement.
     * @return Command-line usage for the rospack tool.
     */
    virtual const char* usage();
};

/**
 * @brief Stack crawler.  Create one of these to operate on a stack
 * tree.  Call public methods inherited from Rosstackage.
 */
class Rosstack : public Rosstackage
{
  public:
    /**
     * @brief Constructor
     */
    Rosstack();
    /**
     * @brief Usage statement.
     * @return Command-line usage for the rosstack tool.
     */
    virtual const char* usage();
};

} // namespace rospack

#endif
