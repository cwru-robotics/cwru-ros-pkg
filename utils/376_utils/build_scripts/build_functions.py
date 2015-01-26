import os
import subprocess
import functools
import sys
import types

# search for notify-send, fortune, cowsay
with open(os.devnull, 'w') as fnull:
    _notify_present = (subprocess.call(['which', 'notify-send'],
                                               stdout=fnull) == 0)
    _cowsay_present = (subprocess.call(['which', 'cowsay'],
                                               stdout=fnull) == 0)

def cowsay(msg):
    p = subprocess.Popen(['cowsay', msg], stdout=subprocess.PIPE)
    out, err = p.communicate()
    return out


def notify_send(title, msg, transient=True):
    if transient:
        subprocess.call(['notify-send',
                         '--hint=int:transient:1',
                         '--icon=emblem-urgent',
                         title,
                         msg])
    else:
        subprocess.call(['notify-send',
                         '--icon=emblem-urgent',
                         title,
                         msg])


def notify(title, msg="", success=True):
    if title is None or title == "":
        title = "<MISSING MESSAGE IN NOTIFY>"
    if msg is None or msg == "":
        msg = title
    print title
    if _cowsay_present:
        msg = cowsay(msg)
    print msg
    if _notify_present:
        notify_send(title, msg)


# TODO capture errors
def run_cmd(working_dir, command):
    assert(type(working_dir) is types.StringType)
    command = " ".join(command)
    # print(command)
    p = subprocess.Popen(command,
                         cwd=working_dir,
                         stdout=subprocess.PIPE,
                         stderr=subprocess.PIPE,
                         shell=True)
    p.communicate()
    return p.returncode == 0


def get_package_path(name):
    p = subprocess.Popen(['rospack', 'find', name], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out, err = p.communicate()
    if p.returncode == 0:
        return out.strip()
    else:
        return None


def get_package_deps(package_name):
    p = subprocess.Popen(['rospack', 'deps', package_name],
                         stdout=subprocess.PIPE)
    deps = []
    for pkg in p.stdout:
        deps.append(pkg[:-1])
    return deps

    
class Package:
    """class representing ros packages, mostly so we don't keep searching for them"""
    CATKIN = 0
    ERROR = -1
    def __init__(self, name):
        self.name = name
        self.path = get_package_path(name)
        self.type = Package.ERROR
        self.deps = None
        if self.path is not None:
            dirlist = os.listdir(self.path)
            if 'package.xml' in dirlist:
                self.type = Package.CATKIN

    def __eq__(self, other):
        # Nothing sensible to be done if type, path or deps don't match
        return self.name == other.name
    
    def __hash__(self):
        # Objects which compare the same must have same hash value, so hash based on name only
        return hash(self.name)


    def get_deps(self):
        if self.deps is None and self.type is not Package.ERROR:
            self.deps = [Package(p) for p in get_package_deps(self.name)]
        return self.deps


# From StackOverflow. Walk to a certain specified depth
def walk_max_depth(some_dir, level=None):
    some_dir = some_dir.rstrip(os.path.sep)
    assert os.path.isdir(some_dir)
    num_sep = some_dir.count(os.path.sep)
    for root, dirs, files in os.walk(some_dir):
        yield root, dirs, files
        num_sep_this = root.count(os.path.sep)
        if level is not None and num_sep + level <= num_sep_this:
            del dirs[:]

# TODO don't like the behavior when type=None
def bash_find(start_dir, name, type='f', max_depth=None):
    for root, dirs, files in walk_max_depth(start_dir, max_depth):
        if type == 'f':
            if name in files:
                return root
        else:
            if name in dirs:
                return root
    return None

# Equivalent to sourcing the given bash script
def bash_source(filename):
    command = ['bash', '-c', 'source %s && env' % filename]
    p = subprocess.Popen(command, stdout=subprocess.PIPE)

    for line in p.stdout:
        (key, _, value) = line.partition('=')
        os.environ[key] = value[:-1]


def get_team_repo():
  return bash_find(os.environ['ROS_WORKSPACE'], '.hku_repository', max_depth=2)


# This is weird right now but note that if with_deps is true we assume packages contains the deps
# and it doesn't otherwise
def catkin_make(team_repo_dir, packages=None, with_deps=False, debug=False):
    if packages is not None and len(packages) == 0:
        return True

    catkin_workspace = os.path.join(team_repo_dir, 'catkin')
    if not os.path.exists(os.path.join(catkin_workspace,
                                       'src/CMakeLists.txt')):
        notify('No catkin workspace found', success=False)
        return False

    print("Catkin workspace: " + catkin_workspace)

    dep_packages = []
    if packages is not None and with_deps is False:
        for package in packages:
            dep_packages += package.get_deps()

    print("Dependencies: " + " ".join([p.name for p in dep_packages]))

    del os.environ['CMAKE_PREFIX_PATH']

    # preserve env variables from our bashrc that will be overwritten
    ros_workspace = os.environ['ROS_WORKSPACE']
    gazebo_model_path = os.environ['GAZEBO_MODEL_PATH']

    bash_source('/opt/ros/hydro/setup.sh')
    bash_source('/usr/share/drcsim/setup.sh')

    build_success = False
    catkin_command = None

    # Whitelist must include deps
    # If with_deps is True, we expect deps have already been included in package list
    # Otherwise need to fetch them
    catkin_args = []
    cmake_args = []
    if packages is not None:
        whitelist_packages = None
        if with_deps:
           whitelist_packages = ';'.join([p.name for p in packages])
        else:
           whitelist_packages = ';'.join([p.name for p in set(packages + dep_packages)])
        build_packages = [p.name for p in packages]
        catkin_args = ["--pkg"] + build_packages
        cmake_args = ["-DCATKIN_WHITELIST_PACKAGES=%s" % whitelist_packages]
    else:
        cmake_args = ["-UCATKIN_WHITELIST_PACKAGES"]

    build_type = "Debug" if debug else "RelWithDebInfo"
  
    command_list = ["catkin_make", "--directory", catkin_workspace] + catkin_args + ["--cmake-args"] + cmake_args + ["-DCMAKE_BUILD_TYPE=" + build_type]

    print(" ".join(command_list))


    p = subprocess.Popen(["pwd"],
                         cwd=catkin_workspace,
                         shell=False)
    build_success = (p.wait() == 0)

    p = subprocess.Popen(command_list,
                         cwd=catkin_workspace,
                         shell=False)
    build_success = (p.wait() == 0)

    if not build_success:
        return False

    bash_source(os.path.join(catkin_workspace, 'devel/setup.sh'))

    # restore env variables
    os.environ['ROS_WORKSPACE'] = ros_workspace
    os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path
    os.environ['ROS_PACKAGE_PATH'] = '%s:%s' % (
        team_repo_dir,
        os.environ['ROS_PACKAGE_PATH'])

    return build_success


def catkin_clean(team_repo_dir, packages=None):
    if packages is not None and len(packages) == 0:
        return True

    # TODO ignore stdout/stderr

    if packages is None:
        run_cmd(team_repo_dir, ['rm', '-rf', 'catkin/build'])
        run_cmd(team_repo_dir, ['rm', '-rf', 'catkin/devel'])
        run_cmd(team_repo_dir, ['rm', '-rf', 'catkin/install'])
    else:
        # delete any directory with the package name under catkin/build
        for package in packages:
            run_cmd(os.path.join(team_repo_dir, 'catkin/build'), ['find', '.', '-type', 'd', '-name', package.name, '-exec', 'rm', '-r', '{}', '\;'])
    return True


def filter_package_list(team_repo_dir, packages, with_deps=False):
    if packages is None or len(packages) == 0:
        return None
   
    catkin_packages = []
    unknown_packages = []
  
    # TODO store Package() instances by name to avoid double lookup of deps
    if with_deps:
        for package_name in set(packages):
            packages += get_package_deps(package_name)
    
    for package_name in set(packages):
        package = Package(package_name)
        if package.type is Package.CATKIN:
            catkin_packages.append(package)
        else:
            unknown_packages.append(package)
    return (catkin_packages, unknown_packages)


def hku_clean(packages, with_deps=False):
    team_repo_dir = get_team_repo()
    
    # TODO make sure package path is set correctly first!
    
    res = filter_package_list(team_repo_dir, packages, with_deps)
    if res is None:
        print("Cleaning all")
        if not catkin_clean(team_repo_dir):
            notify("Failed to clean catkin packages!")
            return False
    else:
        catkin_packages, unknown_packages = res
        if len(unknown_packages) > 0:
            print("Error: unknown packages " + " ".join([p.name for p in unknown_packages]))
            return False
        print("Catkin: " + " ".join([p.name for p in catkin_packages]))
        if not catkin_clean(team_repo_dir, catkin_packages):
            notify("Failed to clean catkin packages!")
            return False
    return True

def hku_make(packages, with_deps=False, debug=False):
    team_repo_dir = get_team_repo()
    
    # TODO make sure package path is set correctly first!
    
    res = filter_package_list(team_repo_dir, packages, with_deps)
    if res is None:
        print("Making all")
        if not catkin_make(team_repo_dir, None, with_deps, debug):
            notify("Failed to build catkin packages!")
            return False
    else:
        catkin_packages, unknown_packages = res
        if len(unknown_packages) > 0:
            print("Error: unknown packages " + " ".join([p.name for p in unknown_packages]))
            return False
        print("Catkin: " + " ".join([p.name for p in catkin_packages]))
        if not catkin_make(team_repo_dir, catkin_packages, with_deps, debug):
            notify("Failed to build catkin packages!")
            return False
    notify("Success!")
    return True
