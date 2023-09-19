import os

import requests
from flask import Flask, request, send_from_directory, Response

app = Flask(__name__)

DEBUG = os.getenv("DEBUG", False) is not False
PORT = int(os.getenv("PORT", "8080"))
HTML_DIR = os.path.join(app.root_path, "html")


# add assets/ to static html/assets
@app.route("/assets/<path:path>")
def send_assets(path):
    return send_from_directory(os.path.join(HTML_DIR, "assets"), path)


# and lib/
@app.route("/lib/<path:path>")
def send_lib(path):
    return send_from_directory(os.path.join(HTML_DIR, "lib"), path)


# add /2d-stream
@app.route("/2d-stream")
def send_2d_stream():
    return send_from_directory(HTML_DIR, "2d-stream.html")


# add /3d-stream
@app.route("/3d-stream")
def send_3d_stream():
    return send_from_directory(HTML_DIR, "3d-stream.html")


# add /3d-adsb
@app.route("/3d-adsb")
def send_3d_adsb():
    return send_from_directory(HTML_DIR, "3d-adsb.html")

# add /basic-stream
@app.route("/basic-stream")
def send_basic_stream():
    return send_from_directory(HTML_DIR, "basic-stream.html")

# add 2d-stream-controls
@app.route("/2d-stream-controls")
def send_2d_stream_controls():
    return send_from_directory(HTML_DIR, "2d-Stream-With-Controls.html")

@app.route("/")
def send_index():
    return send_from_directory(HTML_DIR, "index.html")


# add /api/adsb
@app.route("/api/adsb")
def api_adsb():
    # take ?lan &lon &dist from url
    lat = request.args.get("lat", default=52, type=float)
    lon = request.args.get("lon", default=5, type=float)
    dist = request.args.get("dist", default=250, type=float)

    # get data from adsb.lol
    headers = {
        "user-agent": "github.com/bobcamera/bobcamera",
    }
    R = requests.get(
        f"https://api.adsb.lol/v2/lat/{lat}/lon/{lon}/dist/{dist}",
        headers=headers,
    )
    R.raise_for_status()
    # add header CORS: *
    res = Response(R.text)
    res.headers["Access-Control-Allow-Origin"] = "*"
    # json
    res.headers["Content-Type"] = "application/json"
    return res


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=PORT, debug=DEBUG)
