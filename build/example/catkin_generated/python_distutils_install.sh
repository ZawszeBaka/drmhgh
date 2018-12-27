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
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/yus/Documents/_tmp/_ros_tmp/src/example"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/yus/Documents/_tmp/_ros_tmp/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/yus/Documents/_tmp/_ros_tmp/install/lib/python2.7/dist-packages:/home/yus/Documents/_tmp/_ros_tmp/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/yus/Documents/_tmp/_ros_tmp/build" \
    "/usr/bin/python2" \
    "/home/yus/Documents/_tmp/_ros_tmp/src/example/setup.py" \
    build --build-base "/home/yus/Documents/_tmp/_ros_tmp/build/example" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/yus/Documents/_tmp/_ros_tmp/install" --install-scripts="/home/yus/Documents/_tmp/_ros_tmp/install/bin"
