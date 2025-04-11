This package is to ensure consistent versions of python packages are automatically available across installs

python dependencies are listed in requirements.txt

cuda python dependencies are listed in requirements-cuda.txt


`virtualenv_build.sh` - runs at build time to create the virtual environment and install all the packages listed in requirements.txt. Works by overlaying so rospy etc are still available

`virtualenv_source.sh` - sources the virualenv when the workspace is sourced via catkin env hooks

test it has worked by inspecting the paths:

`echo $PYTHONPATH`

`python -c "import sys; print sys.path"`

`python -c "import numpy as np; print np.__file__"`
