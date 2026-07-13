import esphome.codegen as cg

CODEOWNERS = ["@DT-art1", "@bdraco"]

camera_ns = cg.esphome_ns.namespace("camera")
Camera = camera_ns.class_("Camera", cg.EntityBase, cg.Component)
CameraImage = camera_ns.class_("CameraImage")
CameraImageReader = camera_ns.class_("CameraImageReader")
CameraListener = camera_ns.class_("CameraListener")
CameraRequester = camera_ns.enum("CameraRequester")
Buffer = camera_ns.class_("Buffer")
Encoder = camera_ns.class_("Encoder")
EncoderBuffer = camera_ns.class_("EncoderBuffer")
