# Containerised ros-mesh-preprocessor CLI.
#
#   docker build -t ros-mesh-preprocessor .
#   docker run --rm -v "$PWD:/work" ros-mesh-preprocessor --input part.stl --output part_small.stl --triangles 5000
FROM python:3.12-slim

# System libraries Open3D's wheel links against, even when used headless.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       libegl1 libgl1 libx11-6 libudev1 libusb-1.0-0 libidn2-0 libgomp1 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/ros-mesh-preprocessor
COPY pyproject.toml README.md LICENSE ./
COPY optimizer/ optimizer/
RUN pip install --no-cache-dir .

WORKDIR /work
ENTRYPOINT ["ros-mesh-preprocessor"]
CMD ["--help"]
