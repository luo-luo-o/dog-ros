FROM ros:humble-ros-base AS ros-policy
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8 PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip \
    && rm -rf /var/lib/apt/lists/*
RUN python3 -m pip install --no-cache-dir --only-binary=:all: numpy==1.26.4 onnxruntime==1.20.1
WORKDIR /workspace
ENV DOG_INSTALL=/opt/dog/install DOG_POLICY_DIR=/workspace/policies/deploy
ENTRYPOINT ["/bin/bash", "/usr/local/bin/dog-ros-entrypoint", "--build"]

FROM ros-policy AS hardware
CMD ["ros2", "run", "dog_hardware", "hardware"]

FROM ros-policy AS sim
ENV MUJOCO_GL=glfw DOG_MOTION_DIR=/workspace/motions/dog
RUN apt-get update && apt-get install -y --no-install-recommends libgl1 libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*
RUN python3 -m pip install --no-cache-dir --only-binary=:all: mujoco==3.3.7
CMD ["ros2", "run", "dog_sim", "simulate"]
