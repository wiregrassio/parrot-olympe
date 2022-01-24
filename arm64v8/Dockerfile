FROM arm64v8/ubuntu:20.04 as builder

RUN apt-get update \
    && apt-get upgrade -y \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y libgl1 curl sudo tzdata \
    && printf "password1234\npassword1234\n" | adduser builduser \
    && usermod -aG sudo builduser

USER builduser 

RUN mkdir -p ~/code/parrot-olympe \
    && curl -L https://github.com/Parrot-Developers/olympe/releases/download/v7.0.3/parrot-olympe-src-7.0.3.tar.gz | tar zxf - -C ~/code/parrot-olympe --strip-components=1 \
    && cd ~/code/parrot-olympe \
    && printf "password1234\n" | sudo -S ~/code/parrot-olympe/products/olympe/linux/env/postinst \
    && ~/code/parrot-olympe/build.sh -p olympe-linux -t images -j

FROM arm64v8/python:3.9.9-slim-bullseye

COPY --from=builder /home/builduser/code/parrot-olympe/out/olympe-linux/images /tmp/wheels

RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y libgl1 \
    && cd /tmp/wheels \
    && pip install $(ls) \
    && pip install protobuf haversine \
    && adduser olympe 

USER olympe

#ENTRYPOINT ["python3"]