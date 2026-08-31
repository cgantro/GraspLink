# Platforms

Windows uses Winsock and `network_proxy`. Linux uses POSIX sockets and may use `tc netem` with the required network-administration permission. GLFW/GLAD/GLM are fetched for the Viewer. OpenCV is optional: without it, synthetic streaming and all common tests still build.
