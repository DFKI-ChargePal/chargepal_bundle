#!/usr/bin/env python3

from http.server import BaseHTTPRequestHandler, HTTPServer
import rospy
import rospkg
import time

rospack = rospkg.RosPack()
log_file_path = rospack.get_path("chargepal_bundle") + "/logs/chargepal_logs.txt"


class TxtHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-type", "ChargePal_LOGS/html")
            self.end_headers()

            self.wfile.write(b"<html><head><title>Log Data</title></head><body><pre>")

            try:
                while True:
                    with open(log_file_path, "r") as file:
                        txt_data = file.read()

                    self.wfile.write(bytes(txt_data, "utf-8"))
                    self.wfile.flush()

                time.sleep(0.5)

            except KeyboardInterrupt:
                pass

            self.wfile.write(b"</pre></body></html>")

        else:
            self.send_error(404)


def run_server(server_class=HTTPServer, handler_class=TxtHandler, port=8000):
    server_address = ("", port)
    httpd = server_class(server_address, handler_class)
    print(f"Starting server on port {port}")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass

    httpd.server_close()
    print("Server stopped.")


if __name__ == "__main__":
    try:
        rospy.init_node("txt_data_server", anonymous=True)
        run_server()
    except KeyboardInterrupt:
        pass
