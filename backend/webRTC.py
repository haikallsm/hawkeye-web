import asyncio
import logging
import cv2
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack, RTCIceServer, RTCConfiguration
from av import VideoFrame
from flask import Blueprint, jsonify, request


logging.basicConfig(
    level = logging.INFO,
    format = "%(asctime)s [%(name)s] %(message)s",
)
logger = logging.getLogger("webrtc")

webrtc_bp = Blueprint("webrtc", __name__)

pcs = set()

CAMERA_DEVICE_ID = 0          # /dev/video0 -- ganti kalau webcam ada di index lain
CAPTURE_WIDTH = 640
CAPTURE_HEIGHT = 480
CAPTURE_FPS = 20              # Pi (terutama Pi 3/Zero) berat kalau encode VP8 software di atas ~20-24fps


class WebcamVideoTrack(VideoStreamTrack):

    kind = "video"

    def __init__(self, device_id: int = CAMERA_DEVICE_ID):
        super().__init__()
        self.cap = cv2.VideoCapture(device_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAPTURE_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, CAPTURE_FPS)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            raise RuntimeError(f"Tidak bisa membuka kamera di /dev/video{device_id}")

    async def recv(self):
        pts, time_base = await self.next_timestamp()

        loop = asyncio.get_event_loop()
        ret, frame = await loop.run_in_executor(None, self.cap.read)

        if not ret or frame is None:
            raise RuntimeError("Gagal membaca frame dari webcam (device terlepas / sibuk?)")

        video_frame = VideoFrame.from_ndarray(frame, format="bgr24")
        video_frame.pts = pts
        video_frame.time_base = time_base
        return video_frame

    def stop(self):
        super().stop()
        if self.cap is not None:
            self.cap.release()
            self.cap = None

ICE_SERVERS = [
    RTCIceServer(urls=["stun:stun.l.google.com:19302"]),
]

@webrtc_bp.route("/webrtc/offer", methods=["POST"])
async def webrtc_offer():
    try:
        params = request.get_json(force=True)
        if not params or "sdp" not in params or "type" not in params:
            return jsonify({"status": "error", "message": "Payload SDP tidak lengkap"}), 400

        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

        pc = RTCPeerConnection(configuration=RTCConfiguration(iceServers=ICE_SERVERS))
        pcs.add(pc)

        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            logger.info("Status koneksi WebRTC: %s", pc.connectionState)
            if pc.connectionState in ("failed", "closed", "disconnected"):
                await pc.close()
                pcs.discard(pc)

        @pc.on("iceconnectionstatechange")
        async def on_iceconnectionstatechange():
            logger.info("Status ICE WebRTC: %s", pc.iceConnectionState)

        try:
            video_track = WebcamVideoTrack()
        except RuntimeError as cam_err:
            await pc.close()
            pcs.discard(pc)
            return jsonify({"status": "error", "message": str(cam_err)}), 500

        pc.addTrack(video_track)

        await pc.setRemoteDescription(offer)
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        while pc.iceGatheringState != "complete":
            await asyncio.sleep(0.1)

        for line in pc.localDescription.sdp.split("\r\n"):
            if line.startswith("a=candidate:"):
                logger.info("ICE Candidate: %s", line)

        return jsonify({
            "sdp": pc.localDescription.sdp,
            "type": pc.localDescription.type,
        })

    except Exception as e:
        logger.exception("WebRTC offer gagal diproses")
        return jsonify({"status": "error", "message": str(e)}), 500


async def cleanup_all_peers():
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()

if __name__ == "__main__":
    from flask import Flask
    from flask_cors import CORS

    app = Flask(__name__)
    CORS(app, resources={r"/webrtc/*": {"origins": "*"}})
    app.register_blueprint(webrtc_bp)

    print("Cek IP LAN laptop ini dengan 'ipconfig' (Windows) / 'ifconfig' atau 'ip addr' (Linux/Mac)")
    print("Lalu akses dari Laptop B ke: http://<IP-LAN-laptop-ini>:5000/webrtc/offer")
    app.run(host="0.0.0.0", port=5000, debug=True)