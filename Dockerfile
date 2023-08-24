FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

####
# Machine Setup
####
RUN apt-get update \
    && apt-get install --no-install-recommends --quiet --assume-yes \
    git \
    gcc \
    g++ \
    gdb \
    valgrind \
    patch \
    ssh \
    sudo \
    # ns3 dependencies
    python3 \
    python-is-python3 \
    python3-dev \
    python3-pip \
    python3-pybindgen \
    python3-pygccxml \
    pkg-config \
    sqlite3 \
    #libgtk-3-dev \
    #libsqlite3-dev \
    # pybindgen dependencies
    #libboost-dev \
    #castxml \
    # IoD_Sim dependencies
    rapidjson-dev \
    libgsl-dev \
    libxml2-dev \
    # Fork dependencies
    libboost-all-dev \
    wget \
    nlohmann-json3-dev

RUN wget https://github.com/CrowCpp/Crow/releases/download/v1.0%2B5/crow-v1.0+5.deb \
    && apt install ./crow-v1.0+5.deb

# Instal IoD_Sim
# - Option 1:
RUN git clone https://github.com/LABORA-INF-UFG/IoD_Sim
WORKDIR /IoD_Sim
# - Option 2:
# WORKDIR /IoD_Sim
# COPY . .

RUN ./tools/install-dependencies.sh
RUN ./tools/prepare-ns3.sh
RUN cd ns3/ \
    && ./ns3 configure --build-profile=debug --enable-examples --disable-mpi --disable-python --enable-modules=iodsim \
    && ./ns3 build

CMD [ "/IoD_Sim/ns3/ns3", "run", "iodsim" ]
