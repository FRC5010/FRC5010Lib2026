FROM mcr.microsoft.com/devcontainers/java:17-bullseye

# xvfb provides a virtual display so the WPILib sim GUI (which tries to open
# a window) doesn't crash when you run `xvfb-run ./gradlew simulateJava`.
# libgl1-mesa-dev / libglu1-mesa are pulled in for OpenGL-based WPILib native libs.
RUN apt-get update && apt-get install -y --no-install-recommends \
        xvfb \
        libgl1-mesa-dev \
        libglu1-mesa \
        x11-utils \
    && rm -rf /var/lib/apt/lists/*
