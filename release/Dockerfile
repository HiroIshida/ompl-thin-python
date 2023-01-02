# For better backword compatibility...
FROM ubuntu:18.04
ENV DEBIAN_FRONTEND=noninteractive

RUN apt update
# install pyenv deps
RUN apt-get install -y make build-essential libssl-dev zlib1g-dev libbz2-dev libreadline-dev libsqlite3-dev wget curl llvm libncurses5-dev libncursesw5-dev xz-utils tk-dev libffi-dev liblzma-dev python-openssl git sudo

RUN \
  useradd user && \
  echo "user ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/user && \
  chmod 0440 /etc/sudoers.d/user && \
  mkdir -p /home/user && \
  chown user:user /home/user && \
  chsh -s /bin/bash user

WORKDIR /home/user
USER user
CMD /bin/bash

# build boost static library with fpic
RUN \
    wget -q https://boostorg.jfrog.io/artifactory/main/release/1.71.0/source/boost_1_71_0.tar.gz && \
    tar xf boost_1_71_0.tar.gz && cd boost_1_71_0 && \
    ./bootstrap.sh && \
    sudo ./b2  --with-filesystem --with-serialization --with-program_options --with-system cxxflags=-fPIC cflags=-fPIC link=static -a install
# debug show boost libraries
RUN \
    ls -alt /usr/local/lib

# install pyenv and pythons
ENV HOME /home/user
ENV PYENV_ROOT $HOME/.pyenv
ENV PATH $PYENV_ROOT/shims:$PYENV_ROOT/bin:$PATH
RUN \
  curl https://pyenv.run|bash && \
  echo 'eval "$(pyenv init -)"' >> ~/.bashrc
COPY python_versions.txt $HOME/
RUN  cat python_versions.txt | while read version; do pyenv install $version ; done

# update cmake to 3.20
RUN sudo apt-get remove cmake
RUN wget -nv https://cmake.org/files/v3.20/cmake-3.20.0-rc1-linux-$(uname -m).tar.gz
RUN tar -xf cmake-3.20.0-rc1-linux-$(uname -m).tar.gz
RUN sudo ln -sf ~/cmake-3.20.0-rc1-linux-$(uname -m)/bin/* /usr/bin

# clone and edit file
# insert set(Boost_USE_STATIC_LIBS ON) using sed, because there is no way to set cmake args from pip install. (or exists?)
RUN \
    git clone https://github.com/HiroIshida/ompl-thin-python.git && \
    cd ompl-thin-python && \
    git submodule update --init && \
    sed -i '/find_package(Boost REQUIRED COMPONENTS serialization)/i set(Boost_USE_STATIC_LIBS ON)\n' CMakeLists.txt

RUN \
    sudo apt install libeigen3-dev -y && \
    cd ompl-thin-python && \
    cat ../python_versions.txt | while read version; do pyenv global $version && pip3 install scikit-build && python setup.py bdist_wheel -p manylinux2014_$(uname -m); done

RUN pyenv global $(tail -n1 python_versions.txt)
RUN pip install twine
