import os
import json
import polymetis

__version__ = ""

# Conda installed: Get version of conda pkg (assigned $GIT_DESCRIBE_NUMBER during build)
if "CONDA_PREFIX" in os.environ and os.environ["CONDA_PREFIX"] in polymetis.__file__:
    # Search conda pkgs for polymetis & extract version number
    stream = os.popen("conda list | grep polymetis")
    for line in stream:
        info_fields = [s for s in line.strip("\n").split(" ") if len(s) > 0]
        if info_fields[0] == "polymetis":  # pkg name == polymetis
            __version__ = info_fields[1]
            break

# Built locally: Retrieve git tag description of Polymetis source code
else:
    try:
        # Navigate to polymetis pkg dir, which should be within the git repo
        original_cwd = os.getcwd()
        os.chdir(os.path.dirname(polymetis.__file__))
        
        # Git describe output
        stream = os.popen("git describe --tags")
        version_lines = [line for line in stream]
        
        if version_lines:
            version_string = version_lines[0]
            # Modify to same format as conda env variable GIT_DESCRIBE_NUMBER
            version_items = version_string.strip("\n").split("-")
            if len(version_items) >= 2:
                __version__ = f"{version_items[-2]}_{version_items[-1]}"
            else:
                # Fallback if git describe doesn't have expected format
                __version__ = "0.2.0-dev"
        else:
            # No git tags found (common in forks)
            __version__ = "0.2.0-dev"
        
        # Reset cwd
        os.chdir(original_cwd)
    except Exception as e:
        # Fallback for any errors
        __version__ = "0.2.0-dev"
        os.chdir(original_cwd)

if not __version__:
    __version__ = "0.2.0-dev"
