FROM mcr.microsoft.com/devcontainers/java:17-bookworm

# xvfb provides a virtual display so the WPILib sim GUI (which tries to open
# a window) doesn't crash when you run `xvfb-run ./gradlew simulateJava`.
# libgl1-mesa-dev / libglu1-mesa are pulled in for OpenGL-based WPILib native libs.
# The base image may include a Yarn apt source with a missing signing key.
# Remove it so apt-get update can proceed using Debian repos.
RUN rm -f /etc/apt/sources.list.d/yarn.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        xvfb \
        libgl1-mesa-dev \
        libglu1-mesa \
        x11-utils \
    && rm -rf /var/lib/apt/lists/*
