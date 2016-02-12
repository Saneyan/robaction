FROM ubuntu:16.04
MAINTAINER TADOKORO Saneyuki <saneyan@gfunction.com>

RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y curl git build-essential \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /var/tmp/* /tmp/*

WORKDIR /tmp

RUN git clone http://www.roboken.iit.tsukuba.ac.jp/platform/repos/libscip2awd.git \
    && git clone http://www.roboken.iit.tsukuba.ac.jp/platform/repos/yp-robot-params.git \
    && git clone http://www.roboken.iit.tsukuba.ac.jp/platform/repos/yp-spur.git

RUN cd ./libscip2awd; ./configure && make all install
RUN cd ./yp-robot-params; ./configure && make all install
RUN cd ./yp-spur; ./configure && make all install

COPY ./entry.sh  /sbin/entry.sh
RUN chmod 755 /sbin/entry.sh

ENTRYPOINT ["/sbin/entry.sh"]
CMD ["start"]
