# Tutorial Workspace for MCLR

This repo hosts the implementation of tutorials for "Modelling and Control of Legged Robots" course.

Clone the repo:

```bash
git clone https://github.com/Jonas-Zhang97/TUM-MCLR.git
```

If you are familiar with devcontainer, you can skip this steps, direct to this [page](https://github.com/Jonas-Zhang97/TUM-MCLR/tree/master/mclr_ws/src) or this [readme](./mclr_ws/src/Readme.md) to ckeck out the code.

## Prerequest

- Enable remote development extension in your VSCode (extension ID: ms-vscode-remote.vscode-remote-extensionpack).
- It is recommended to use the docker container in a computer with NVidia GPU, to start directly with GPU, you can skip to the [get started](#get-started), if not available, follow this [chapter](#build-without-gpu) to build the docker container without GPU

## Build without GPU

Navigate to the `.devcontainer/Dockerfile`, in stage 4, comment the following lines:

```dockerfile
ENV NVIDIA_VISIBLE_DEVICES \
  ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
  ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
```

then, to the file `.devcontainer/devcontainer.json`, in `runArgs` comment the following lines:

```json
"--runtime=nvidia"
"--gpus=all"
```

## Get Started

Open the local repo in VSCode, you should see a pop-up asking if you want to reopen in devcontainer, click yes. If you don't see any pop-up, press <kbd>ctrl</kbd> + <kbd>shift</kbd> + <kbd>p</kbd>, type "remote" in searchbar and choose `rebuild and reopen in devcontainer`.

After the container is built, you can then check this [page](https://github.com/Jonas-Zhang97/TUM-MCLR/tree/master/mclr_ws/src) or this [readme](./mclr_ws/src/Readme.md) for the code.

## Issues

- [ ] FromAsCasing issue in Dockerfile (not fetal, fix in the end)