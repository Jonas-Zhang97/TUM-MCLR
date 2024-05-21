# simulator

Robot Simulator based on pybullet and pinocchio

## Installation

Add robotpkg as source repo to apt

```bash
sudo sh -c "echo 'deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg' >> /etc/apt/sources.list.d/robotpkg.list"
```

Register the authentication certificate of robopkg

```bash
sudo apt-get update
```

Then update apt to fetch

```bash
sudo apt-get update
```

```bash
sudo apt install robotpkg-py38-qt5-gepetto-viewer-corba
sudo apt-get install python-gi-cairo
sudo apt install robotpkg-py38-example-robot-data
```

## Links

* [TSID Git](https://github.com/stack-of-tasks/tsid/wiki)
* [TSID Doxy](https://gepettoweb.laas.fr/doc/stack-of-tasks/tsid/master/doxygen-html/namespacetsid_1_1tasks.html)
