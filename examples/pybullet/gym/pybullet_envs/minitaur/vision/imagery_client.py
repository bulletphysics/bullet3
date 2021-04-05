"""The imagery client to connect to the camera job."""

from typing import Any, Dict, Sequence, Text
import gin

from pybullet_envs.minitaur.fw_bridge import worker_builder
from pybullet_envs.minitaur.vision import imagery_pb2
from pybullet_envs.minitaur.vision import imagery_utils
from google3.third_party.fluxworks.core.fluxworks.python.genericutil_py import fwassert
from google3.third_party.fluxworks.core.fluxworks.python.genericutil_py import timeutil

_RPC_TIMEOUT = 1 * timeutil.TimeUtil.SEC

_URI_START_CAPTURE = "fwuri://VisionJob/StartCapture"
_URI_STOP_CAPTURE = "fwuri://VisionJob/StopCapture"
_URI_GET_FRAME = "fwuri://VisionJob/GetFrame"


@gin.configurable
class ImageryClient(object):
  """Sends commands and receives states from cameras."""

  def __init__(
      self,
      fw_worker=None,
      rpc_timeout_sec=_RPC_TIMEOUT,
      ip_address=None,
      port=None,
      async_mode=False,
      start_capture_uri: Text = _URI_START_CAPTURE,
      stop_capture_uri: Text = _URI_STOP_CAPTURE,
      get_frame_uri: Text = _URI_GET_FRAME,
  ):
    """Initializes the client.

    Args:
      fw_worker: A FluxWorks worker instance.
      rpc_timeout_sec: The timeout for any RPC calls from this client.
      ip_address: The ip address of the camera/vision process. If vision job is
        also instantiated in the same FluxWorks worker, both ip address and port
        are not needed.
      port: The port of the camera/vision process.
      async_mode: Whether the RPC calls in this client are async or synchronous.
        Aync mode is only required when you have multiple workers communicating
        with each other in the same Python process. If worker A is calling
        worker B's RPC, worker B's RPC is trying to get GIL from it's thread but
        caller (worker A) already holds the GIL, and this will cause a dead lock
        if worker A's calls are synchronous. If worker A is calling its own RPC,
        the same GIL can be used so there is no dead lock, and there is no need
        for async mode. Async mode will require context switching and thus is a
        bit slower.
      start_capture_uri: The FluxWorks URI to start camera capture.
      stop_capture_uri: The FluxWorks URI to stop camera capture.
      get_frame_uri: The FluxWorks URI to get camera frames.
    """
    self._rpc_timeout_sec = rpc_timeout_sec
    if fw_worker is None:
      fw_worker = worker_builder.GetDefaultWorker()
    self._worker = fw_worker

    # TODO(tingnan): Use a single address and split the string for FW.
    if ip_address is not None:
      self._worker.ConnectToWorker(ip_address, port)

    self._async_mode = async_mode
    self._start_capture_uri = start_capture_uri
    self._stop_capture_uri = stop_capture_uri
    self._get_frame_uri = get_frame_uri

  def _convert_camera_frame_to_image_dict(
      self, camera_frame: imagery_pb2.CameraFrame):
    """Converts the camera frame to an image dictionary."""
    # Each camera frame might contain multiple image channels, such as rgb and
    # depth.
    images = {}
    for image_name, image_proto in camera_frame.images.items():
      image_array = imagery_utils.convert_image_to_array(image_proto)
      images[image_name] = image_array
    return images

  def start_capture(self, run_id: Text = "vision"):
    """Starts the camera capture session.

    Args:
      run_id: The capture session id. This id will determine the name of the
        image logs' sub-direcotry.
    """
    capture_request = imagery_pb2.CaptureRequest()
    capture_request.run_id = run_id
    fwassert.FwAssert.CheckErrorMessage(
        self._worker.CallOnewayProtoRpc(
            self._start_capture_uri,
            capture_request,
            async_mode=self._async_mode))

  def stop_capture(self):
    """Concludes the current capture session."""
    capture_request = imagery_pb2.CaptureRequest()
    fwassert.FwAssert.CheckErrorMessage(
        self._worker.CallOnewayProtoRpc(
            self._stop_capture_uri,
            capture_request,
            async_mode=self._async_mode))

  def get_camera_images(self) -> Dict[Text, Sequence[Any]]:
    """Gets the latest camera images.

    Camera images can only be obtained after self.start_capture() is called.

    Returns:
      A dictionary of camera frames, with the camera id as the key. Each camera
      frame may contain multiple streams. For example, on a realsense camera we
      may have "rgb" and "depth" streams, depending on the configuration.
    """
    get_frame_request = imagery_pb2.GetFrameRequest()
    frame_collection = imagery_pb2.CameraFrameCollection()
    fwassert.FwAssert.CheckErrorMessage(
        self._worker.CallRoundtripProtoRpc(
            self._get_frame_uri,
            get_frame_request,
            frame_collection,
            self._rpc_timeout_sec,
            async_mode=self._async_mode))

    images_by_camera = {}
    for camera_frame in frame_collection.frames:
      camera_id = camera_frame.camera_id
      # In case we received multiple frames, we apppend them in the order
      # received.
      if camera_id in images_by_camera:
        images_by_camera[camera_id].append(
            self._convert_camera_frame_to_image_dict(camera_frame))
      else:
        images_by_camera[camera_id] = [
            self._convert_camera_frame_to_image_dict(camera_frame)
        ]
    return images_by_camera
