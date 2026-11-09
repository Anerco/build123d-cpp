// export_to_pcbway against a local one-request HTTP server (nothing is sent to PCBWay). The
// expected request has the shape of the one Python's requests.post sends.
#include <build123d/build123d.hpp>

#include "test_framework.hpp"

#if !defined(_WIN32)

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cstring>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>

using namespace build123d;

namespace {

/// Accepts one connection on 127.0.0.1, records the request and sends a canned response.
class MockServer {
public:
    explicit MockServer(std::string response) : response_(std::move(response)) {
        fd_ = socket(AF_INET, SOCK_STREAM, 0);
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port = 0;
        bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
        listen(fd_, 1);
        socklen_t length = sizeof(addr);
        getsockname(fd_, reinterpret_cast<sockaddr*>(&addr), &length);
        port_ = ntohs(addr.sin_port);
        thread_ = std::thread([this] { serve(); });
    }
    MockServer(const MockServer&) = delete;
    MockServer& operator=(const MockServer&) = delete;
    ~MockServer() {
        if (thread_.joinable()) thread_.join();
        close(fd_);
    }
    std::string url() const { return "http://127.0.0.1:" + std::to_string(port_) + "/common/Build123dUpFile"; }
    const std::string& request() {
        if (thread_.joinable()) thread_.join();
        return request_;
    }

private:
    void serve() {
        timeval timeout{30, 0};
        setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        const int client = accept(fd_, nullptr, nullptr);
        if (client < 0) return;
        setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        std::string data;
        std::size_t header_end = std::string::npos, content_length = 0;
        char buffer[65536];
        while (true) {
            const ssize_t n = recv(client, buffer, sizeof(buffer), 0);
            if (n <= 0) break;
            data.append(buffer, static_cast<std::size_t>(n));
            if (header_end == std::string::npos && (header_end = data.find("\r\n\r\n")) != std::string::npos) {
                std::string headers = data.substr(0, header_end);
                std::transform(headers.begin(), headers.end(), headers.begin(),
                               [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                const auto field = headers.find("\r\ncontent-length:");
                if (field != std::string::npos) content_length = std::stoul(headers.substr(field + 17));
            }
            if (header_end != std::string::npos && data.size() >= header_end + 4 + content_length) break;
        }
        request_ = data;
        send(client, response_.data(), response_.size(), MSG_NOSIGNAL);
        shutdown(client, SHUT_WR);
        close(client);
    }

    int fd_ = -1;
    int port_ = 0;
    std::string response_;
    std::string request_;
    std::thread thread_;
};

std::string http_response(const std::string& status, const std::string& body,
                          const std::string& type = "application/json") {
    return "HTTP/1.1 " + status + "\r\nContent-Type: " + type + "\r\nContent-Length: " + std::to_string(body.size()) +
           "\r\nConnection: close\r\n\r\n" + body;
}

PcbwayUpload local_upload(const std::string& url, std::vector<std::string>* opened, bool open_result = true) {
    PcbwayUpload upload;
    upload.upload_url = url;
    upload.open_browser = [opened, open_result](const std::string& page) {
        if (opened != nullptr) opened->push_back(page);
        return open_result;
    };
    return upload;
}

std::string error_of(const PcbwayUpload& upload) {
    try {
        export_to_pcbway(Box(1, 1, 1), upload);
    } catch (const std::runtime_error& e) {
        return e.what();
    }
    return "";
}

}  // namespace

TEST(pcbway_upload) {
    MockServer server(http_response("200 OK", R"({"state": "SUCCESS", "redirect": "https://example.invalid/quote?id=7"})"));
    std::vector<std::string> opened;
    const std::string page = export_to_pcbway(Box(1, 2, 3), local_upload(server.url(), &opened));
    CHECK(page == "https://example.invalid/quote?id=7");
    CHECK(opened == std::vector<std::string>{page});

    const std::string& request = server.request();
    CHECK(request.rfind("POST /common/Build123dUpFile HTTP/1.1\r\n", 0) == 0);
    CHECK(request.find("Content-Type: multipart/form-data; boundary=") != std::string::npos);
    const std::string disposition = "Content-Disposition: form-data; name=\"file\"; filename=\"";
    const auto part = request.find(disposition);
    CHECK(part != std::string::npos);
    const auto name_start = part + disposition.size();
    const std::string file_name = request.substr(name_start, request.find('"', name_start) - name_start);
    CHECK(file_name.size() == 16 && file_name.rfind("tmp", 0) == 0 && file_name.substr(11) == ".step");
    CHECK(request.find("Content-Type: application/step\r\n", part) != std::string::npos);
    CHECK(request.find("ISO-10303-21;") != std::string::npos);
    CHECK(request.find("END-ISO-10303-21;") != std::string::npos);
    // the temporary STEP file is gone
    CHECK(!std::filesystem::exists(std::filesystem::temp_directory_path() / file_name));
}

TEST(pcbway_errors) {
    {
        MockServer server(http_response("500 Internal Server Error", "oops", "text/plain"));
        CHECK(error_of(local_upload(server.url(), nullptr)) ==
              "500 Server Error: Internal Server Error for url: " + server.url());
    }
    {
        MockServer server(http_response("200 OK", "<html>maintenance</html>", "text/html"));
        CHECK(error_of(local_upload(server.url(), nullptr)) ==
              "PCBWay returned a non-JSON response: <html>maintenance</html>");
    }
    {
        MockServer server(http_response("200 OK", R"({"state": "FAIL", "msg": "too big", "redirect": null})"));
        CHECK(error_of(local_upload(server.url(), nullptr)) ==
              "PCBWay upload failed or returned no redirect: {'state': 'FAIL', 'msg': 'too big', 'redirect': None}");
    }
    {
        // nothing listens on this port any more
        int port = 0;
        {
            MockServer server(http_response("200 OK", "{}"));
            port = std::stoi(server.url().substr(17, server.url().find('/', 17) - 17));
            const std::string ignored = error_of(local_upload(server.url(), nullptr));
        }
        const std::string message =
            error_of(local_upload("http://127.0.0.1:" + std::to_string(port) + "/common/Build123dUpFile", nullptr));
        CHECK(message.rfind("Upload to http://127.0.0.1:", 0) == 0);
    }
}

TEST(pcbway_browser_failure_warns) {
    MockServer server(http_response("200 OK", R"({"state": "SUCCESS", "redirect": "https://example.invalid/q"})"));
    std::vector<std::string> warnings;
    const WarningHandler previous = set_warning_handler([&](const std::string& m) { warnings.push_back(m); });
    const std::string page = export_to_pcbway(Box(1, 1, 1), local_upload(server.url(), nullptr, false));
    set_warning_handler(previous);
    CHECK(page == "https://example.invalid/q");
    CHECK(warnings == std::vector<std::string>{"webbrowser failed to open on pricing page https://example.invalid/q"});
    CHECK(!open_in_browser("file:///etc/passwd"));
    CHECK(!open_in_browser("-x"));
}

#endif  // !_WIN32
