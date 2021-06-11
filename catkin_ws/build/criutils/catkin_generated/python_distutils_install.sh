#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/matt/JBOT_Working/catkin_ws/src/criutils"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/matt/JBOT_Working/catkin_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/matt/JBOT_Working/catkin_ws/install/lib/python2.7/dist-packages:/home/matt/JBOT_Working/catkin_ws/build/criutils/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/matt/JBOT_Working/catkin_ws/build/criutils" \
    "/usr/bin/python2" \
    "/home/matt/JBOT_Working/catkin_ws/src/criutils/setup.py" \
     \
    build --build-base "/home/matt/JBOT_Working/catkin_ws/build/criutils" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/matt/JBOT_Working/catkin_ws/install" --install-scripts="/home/matt/JBOT_Working/catkin_ws/install/bin"
