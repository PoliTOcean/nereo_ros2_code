#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cstring>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <endian.h> // htobe64

// Helper: send all bytes
bool sendAll(int sock, const void* data, size_t len) {
    const char* ptr = reinterpret_cast<const char*>(data);
    size_t remaining = len;
    while (remaining > 0) {
        ssize_t sent = send(sock, ptr, remaining, 0);
        if (sent <= 0) return false;
        ptr += sent;
        remaining -= sent;
    }
    return true;
}

int main(int argc, char** argv) {
    // Ensure port matches client configuration
    const int PORT = 9999;
    int server_fd;
    struct sockaddr_in addr{};

    // 1. Create listening socket
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        std::cerr << "Socket creation error" << std::endl;
        return -1;
    }
    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(PORT);

    if (bind(server_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "Bind failed" << std::endl;
        close(server_fd);
        return -1;
    }
    if (listen(server_fd, 1) < 0) {
        std::cerr << "Listen failed" << std::endl;
        close(server_fd);
        return -1;
    }
    std::cout << "Server listening on port " << PORT << std::endl;

    while (true) {
        // Accept a client
        struct sockaddr_in client_addr{};
        socklen_t len = sizeof(client_addr);
        int client_fd = accept(server_fd, (struct sockaddr*)&client_addr, &len);
        if (client_fd < 0) {
            std::cerr << "Accept failed" << std::endl;
            continue;
        }
        std::cout << "Client connected: " << inet_ntoa(client_addr.sin_addr) << std::endl;

        // Open webcam with V4L2 backend to avoid GStreamer warnings
        cv::VideoCapture cap(0, cv::CAP_V4L2);
        if (!cap.isOpened()) {
            std::cerr << "Cannot open webcam" << std::endl;
            close(client_fd);
            continue;
        }

        // Optional: set resolution
        // cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        // cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

        std::vector<uchar> buffer;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80};

        while (true) {
            cv::Mat frame;
            if (!cap.read(frame) || frame.empty()) {
                std::cerr << "Frame capture failed or empty" << std::endl;
                break;
            }
            // Encode JPEG
            buffer.clear();
            if (!cv::imencode(".jpg", frame, buffer, params)) {
                std::cerr << "JPEG encoding failed" << std::endl;
                break;
            }
            // Send length prefix
            uint64_t size = buffer.size();
            uint64_t netSize = htobe64(size);
            if (!sendAll(client_fd, &netSize, sizeof(netSize)) ||
                !sendAll(client_fd, buffer.data(), buffer.size())) {
                std::cerr << "Send failed or client disconnected" << std::endl;
                break;
            }
        }

        std::cout << "Client disconnected or error occurred" << std::endl;
        cap.release();
        close(client_fd);
    }

    close(server_fd);
    return 0;
}

