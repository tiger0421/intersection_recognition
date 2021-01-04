FROM osrf/ros:melodic-desktop-full

# Install MeCab
RUN apt update && \
    apt upgrade -y && \
    apt install -y --no-install-recommends \
        vim \
        wget \
        git \
        curl \
        file \
        swig \
        mecab \
        libmecab-dev \
        mecab-ipadic-utf8 \
        ros-melodic-geometry2 \
        python-catkin-tools \
        python-rosdep \
        python-rosinstall-generator \
        python-wstool \
        python-rosinstall && \
    apt autoremove -y && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

# Install gensim
RUN curl https://bootstrap.pypa.io/get-pip.py | python && \
    pip install \
        gensim \
        mecab-python3

# Install mecab-ipadic-NEologd
RUN mkdir /tmp/neologd \
  && git clone --depth 1 https://github.com/neologd/mecab-ipadic-neologd.git /tmp/neologd \
  && /tmp/neologd/bin/install-mecab-ipadic-neologd -n -a -y \
  && sed -i -e "s|^dicdir.*$|dicdir = /usr/lib/x86_64-linux-gnu/mecab/dic/mecab-ipadic-neologd|" /etc/mecabrc \
  && rm -rf /tmp/neologd

# Install neologdn
RUN pip install neologdn

# Install CRF++
RUN wget -O /tmp/CRF++-0.58.tar.gz "https://drive.google.com/uc?export=download&id=0B4y35FiV1wh7QVR6VXJ5dWExSTQ" \
  && cd /tmp \
  && tar zxf CRF++-0.58.tar.gz \
  && cd CRF++-0.58 \
  && ./configure \
  && make \
  && make install \
  && cd / \
  && rm /tmp/CRF++-0.58.tar.gz \
  && rm -rf /tmp/CRF++-0.58 \
  && ldconfig

# Install CaboCha
RUN cd /tmp \
  && curl -c cabocha-0.69.tar.bz2 -s -L "https://drive.google.com/uc?export=download&id=0B4y35FiV1wh7SDd1Q1dUQkZQaUU" \
    | grep confirm | sed -e "s/^.*confirm=\(.*\)&amp;id=.*$/\1/" \
    | xargs -I{} curl -b  cabocha-0.69.tar.bz2 -L -o cabocha-0.69.tar.bz2 \
      "https://drive.google.com/uc?confirm={}&export=download&id=0B4y35FiV1wh7SDd1Q1dUQkZQaUU" \
  && tar jxf cabocha-0.69.tar.bz2 \
  && cd cabocha-0.69 \
  && export CPPFLAGS=-I/usr/local/include \
  && ./configure --with-mecab-config=`which mecab-config` --with-charset=utf8 \
  && make \
  && make install \
  && cd python \
  && python setup.py build \
  && python setup.py install

RUN ln /etc/mecabrc /usr/local/etc/mecabrc

RUN echo "source /opt/ros/melodic/setup.bash >> /root/.bashrc"
ENV SHELL=/bin/bash
WORKDIR /root
CMD bash
