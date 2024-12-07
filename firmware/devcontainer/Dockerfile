FROM espressif/idf

ARG DEBIAN_FRONTEND=nointeractive
ARG CONTAINER_USER=esp
ARG USER_UID=1005
ARG USER_GID=$USER_UID
ARG USERNAME=$CONTAINER_USER

RUN apt-get update && apt-get install -y \
    apt-utils \
    bison \
    ca-certificates \
    ccache \
    check \
    curl \
    flex \
    git \
    git-lfs \
    gperf \
    lcov \
    libbsd-dev \
    libffi-dev \
    libncurses-dev \
    libpixman-1-0 \
    libslirp0 \
    libusb-1.0-0-dev \
    make \
    ninja-build \
    python3 \
    python3-pip \
    python3-venv \
    python3-pytest \
    qemu-system-xtensa \
    unzip \
    wget \
    xz-utils \
    && rm -rf /var/lib/apt/lists/*

    
#create non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME 
    
RUN apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
    

# ESP-IDF setup
ARG IDF_PATH=/home/esp/idf
ARG IDF_TOOLS_PATH=/esp/idf/tools 
ARG IDF_VERSION=v5.3.2


RUN git clone --recursive --branch $IDF_VERSION https://github.com/espressif/esp-idf.git $IDF_PATH 

RUN cd $IDF_PATH \
    && ./install.sh all

RUN . $IDF_PATH/export.sh && \
    pip3 install pytest-embedded-serial-esp pytest-embedded-idf \
    pytest-embedded-qemu pytest-embedded pytest-rerunfailures

# for convenience
RUN echo "alias get_idf='. $IDF_PATH/export.sh'" >> ~/.bashrc \
    && echo "source $IDF_PATH/export.sh" >> ~/.bashrc \
    && echo "alias run_tests='pytest --embedded-services=esp,qemu'" >> ~/.bashrc \

ENV PATH="${IDF_PATH}/tools/xtensa-esp32-elf/esp-13.2.0_20240530/xtensa-esp32-elf/bin:${PATH}"
ENV PATH="${IDF_PATH}/tools:${PATH}"
ENV IDF_PATH=$IDF_PATH
ENV IDF_TOOLS_PATH=$IDF_TOOLS_PATH
ENV IDF_PYTHON_CHECK_CONSTRAINTS=no
ENV IDF_CCACHE_ENABLE=1


# Set the working directory for all subsequent operations
WORKDIR /workspace

CMD ["/bin/bash", "-c"]

