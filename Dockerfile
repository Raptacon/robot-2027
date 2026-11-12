# Dev container for either RobotPy line. Debian 13 (trixie) has glibc 2.41,
# which the RobotPy 2027 native wheels need. The python images include tkinter.
#   docker build . --tag raptacon2027_build                            # 2026, Python 3.14
#   docker build . --build-arg PYTHON_VERSION=3.15 --tag raptacon2027_build_2027   # 2027 alpha
ARG PYTHON_VERSION=3.14
FROM python:${PYTHON_VERSION}-trixie
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends make vim git \
    && rm -rf /var/lib/apt/lists/*
