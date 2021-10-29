FROM --platform=linux/arm64/v8 tiger0421/ros:noetic-arm

# Install MeCab
RUN apt update && \
    apt upgrade -y && \
    apt install -y --no-install-recommends \
        build-essential \
        vim \
        wget \
        git \
        curl \
        file \
        patch \
        swig \
        mecab \
        libmecab-dev \
        mecab-ipadic-utf8 \
        ros-noetic-geometry2 \
        python3-catkin-tools \
        python3-rosdep \
        python3-rosinstall-generator \
        python3-wstool \
        python3-rosinstall && \
    apt autoremove -y && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

# Install gensim
RUN curl https://bootstrap.pypa.io/get-pip.py | python3 && \
    python3 -m pip install --no-cache-dir \
        gensim \
        mecab-python3

# Install mecab-ipadic-NEologd
RUN mkdir /tmp/neologd \
  && git clone --depth 1 https://github.com/neologd/mecab-ipadic-neologd.git /tmp/neologd \
  && /tmp/neologd/bin/install-mecab-ipadic-neologd -n -a -y \
  && sed -i -e "s|^dicdir.*$|dicdir = /usr/lib/x86_64-linux-gnu/mecab/dic/mecab-ipadic-neologd|" /etc/mecabrc \
  && rm -rf /tmp/neologd

# Install neologdn
RUN pip install --no-cache-dir neologdn

RUN ln /etc/mecabrc /usr/local/etc/mecabrc \
  && ln -n /usr/bin/python3 /usr/bin/python

ENV SHELL=/bin/bash
WORKDIR /root
CMD bash
