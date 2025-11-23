import sys
import gi
from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtWidgets import QLabel, QVBoxLayout, QWidget

gi.require_version('Gst', '1.0')
from gi.repository import Gst

class VideoStreamWindow(QWidget):
    update_signal = pyqtSignal(QImage)

    def __init__(self, port: int):
        super().__init__()

        Gst.init(None)

        self.pipeline = self.create_pipeline(port)

        self.processing_frame = False
        self.latest_image = None

        self.video_label = QLabel(self)

        layout = QVBoxLayout()
        layout.addWidget(self.video_label)
        self.setLayout(layout)

        self.pipeline.set_state(Gst.State.PLAYING)
        self.update_signal.connect(self.update_image)

        # Timer per limitare gli aggiornamenti della GUI, 20 fps
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.check_for_updates)
        self.timer.start(20)

    def create_pipeline(self, port: str):
        """Crea e ritorna il pipeline GStreamer."""
        pipeline_str = f"udpsrc port={port} ! application/x-rtp, payload=96 ! rtph264depay ! avdec_h264 ! videoconvert ! video/x-raw, format=RGB ! appsink name=sink"
        pipeline = Gst.parse_launch(pipeline_str)

        appsink = pipeline.get_by_name("sink")
        appsink.set_property("emit-signals", True)
        appsink.connect("new-sample", self.on_new_sample)

        return pipeline

    def on_new_sample(self, sink):
        """Processa il frame ricevuto dal pipeline GStreamer."""
        try:
            if self.processing_frame:
                return Gst.FlowReturn.OK

            self.processing_frame = True
            sample = sink.emit("pull-sample")
            if not sample:
                self.processing_frame = False
                return Gst.FlowReturn.ERROR

            buffer = sample.get_buffer()
            if not buffer:
                self.processing_frame = False
                self.log_error("Buffer non trovato")
                return Gst.FlowReturn.ERROR

            width, height = self.get_frame_resolution(sample)

            image = self.extract_image_from_buffer(buffer, width, height)

            if image:
                self.latest_image = image
            else:
                self.log_error("Immagine non valida")

            self.processing_frame = False
            return Gst.FlowReturn.OK

        except Exception as e:
            self.processing_frame = False
            self.log_error(f"Errore durante il processing del frame: {e}")
            return Gst.FlowReturn.ERROR

    def get_frame_resolution(self, sample):
        """Estrae la risoluzione del frame dal sample."""
        caps = sample.get_caps()
        width = caps.get_structure(0).get_int("width")[1]
        height = caps.get_structure(0).get_int("height")[1]
        return width, height

    def extract_image_from_buffer(self, buffer, width, height):
        """Estrae un'immagine dal buffer GStreamer."""
        success, map_info = buffer.map(Gst.MapFlags.READ)
        if not success:
            self.log_error("Errore nel mapping del buffer")
            return None

        try:
            if map_info.data:
                image = QImage(bytes(map_info.data), width, height, QImage.Format.Format_RGB888)
                return image
            else:
                self.log_error("I dati del buffer sono nulli!")
                return None
        finally:
            buffer.unmap(map_info)

    def update_image(self, image):
        """Aggiorna il QLabel con il nuovo frame."""
        scaled_image = image.scaled(self.video_label.size(), Qt.AspectRatioMode.KeepAspectRatio)
        self.video_label.setPixmap(QPixmap.fromImage(scaled_image))

    def check_for_updates(self):
        """Controlla se è pronto un nuovo frame e aggiorna la GUI."""
        if self.latest_image:
            self.update_signal.emit(self.latest_image)

    def log_error(self, message):
        """Log degli errori."""
        print(f"Errore: {message}")

    def closeEvent(self, event):
        """Gestisce l'evento di chiusura della finestra."""
        print("Closing pipeline.")
        self.pipeline.set_state(Gst.State.NULL)
        event.accept()