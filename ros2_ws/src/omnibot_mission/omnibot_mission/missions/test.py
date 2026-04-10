import sys
import cv2
import numpy as np
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QLabel, QFileDialog, QPushButton, QVBoxLayout, QWidget, QListWidget, QComboBox, QHBoxLayout, QLineEdit, QMessageBox
)
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import Qt, QPoint

class MapArucoUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Map ArUco UI")
        self.image = None
        self.display_image = None
        self.zero_point = None
        self.aruco_coords = []
        self.points_of_interest = []
        self.aruco_dict = None
        self.aruco_available = hasattr(cv2, 'aruco') and hasattr(cv2.aruco, 'detectMarkers')
        if self.aruco_available:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.coord_system = {'x': 'right', 'y': 'down'}
        self.anchor_type = 'center'
        self.scale = 1.0
        self.init_ui()

    def init_ui(self):
        self.label = QLabel("Загрузите карту", self)
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setFixedSize(800, 600)

        load_btn = QPushButton("Загрузить карту")
        load_btn.clicked.connect(self.load_image)

        aruco_btn = QPushButton("Распознать ArUco")
        aruco_btn.clicked.connect(self.detect_aruco)

        save_btn = QPushButton("Сохранить результаты")
        save_btn.clicked.connect(self.save_results)

        # --- New controls for coordinate system, anchor, scale ---
        coord_layout = QHBoxLayout()
        self.x_dir = QComboBox()
        self.x_dir.addItems(["right", "left"])
        self.x_dir.setCurrentText("right")
        self.x_dir.currentTextChanged.connect(lambda v: self.set_coord_system('x', v))
        coord_layout.addWidget(QLabel("X ось →"))
        coord_layout.addWidget(self.x_dir)

        self.y_dir = QComboBox()
        self.y_dir.addItems(["down", "up"])
        self.y_dir.setCurrentText("down")
        self.y_dir.currentTextChanged.connect(lambda v: self.set_coord_system('y', v))
        coord_layout.addWidget(QLabel("Y ось →"))
        coord_layout.addWidget(self.y_dir)

        self.anchor_box = QComboBox()
        self.anchor_box.addItems(["center", "top-left", "top-right", "bottom-left", "bottom-right"])
        self.anchor_box.setCurrentText("center")
        self.anchor_box.currentTextChanged.connect(self.set_anchor_type)
        coord_layout.addWidget(QLabel("Привязка:"))
        coord_layout.addWidget(self.anchor_box)

        self.scale_edit = QLineEdit("1.0")
        self.scale_edit.setFixedWidth(60)
        self.scale_edit.editingFinished.connect(self.set_scale)
        coord_layout.addWidget(QLabel("Масштаб (м/пикс):"))
        coord_layout.addWidget(self.scale_edit)

        self.list_widget = QListWidget()

        layout = QVBoxLayout()
        layout.addWidget(self.label)
        layout.addLayout(coord_layout)
        layout.addWidget(load_btn)
        layout.addWidget(aruco_btn)
        layout.addWidget(save_btn)
        layout.addWidget(self.list_widget)

        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

    def set_coord_system(self, axis, value):
        self.coord_system[axis] = value

    def set_anchor_type(self, value):
        self.anchor_type = value

    def set_scale(self):
        try:
            self.scale = float(self.scale_edit.text())
        except Exception:
            self.scale = 1.0
            self.scale_edit.setText("1.0")

    def load_image(self):
        fname, _ = QFileDialog.getOpenFileName(self, "Выберите карту", "", "Images (*.tif *.png *.jpg)")
        if fname:
            self.image = cv2.imread(fname)
            if self.image is None:
                QMessageBox.critical(self, "Ошибка", "Не удалось загрузить изображение!")
                return
            self.display_image = self.image.copy()
            self.zero_point = None
            self.aruco_coords = []
            self.points_of_interest = []
            self.list_widget.clear()
            self.label.setText("Кликните для выбора мирового нуля")
            self.update_display()  # Показываем карту сразу
    
    def mousePressEvent(self, event):
        if self.image is None:
            return
        pos = self.label.mapFromParent(event.pos())
        if 0 <= pos.x() < self.label.width() and 0 <= pos.y() < self.label.height():
            x = int(pos.x() * self.image.shape[1] / self.label.width())
            y = int(pos.y() * self.image.shape[0] / self.label.height())
            if self.zero_point is None:
                self.zero_point = (x, y)
                self.label.setText("Мировой ноль выбран. Кликните для добавления интересующих точек.")
            else:
                self.points_of_interest.append((x, y))
                self.list_widget.addItem(f"Точка интереса: {x}, {y}")
            self.update_display()

    def detect_aruco(self):
        if self.image is None:
            return
        if not self.aruco_available:
            QMessageBox.critical(self, "Ошибка", "Модуль aruco не найден в OpenCV! Установите пакет opencv-contrib-python.")
            return
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict)
        self.aruco_coords = []
        self.list_widget.addItem("--- ArUco ---")
        if ids is not None:
            for i, corner in enumerate(corners):
                c = corner[0].mean(axis=0)
                self.aruco_coords.append((int(c[0]), int(c[1]), int(ids[i][0])))
                self.list_widget.addItem(f"ArUco {ids[i][0]}: {int(c[0])}, {int(c[1])}")
        else:
            self.list_widget.addItem("ArUco не найдены")
        self.update_display()

    def update_display(self):
        if self.image is None:
            self.label.clear()
            self.label.setText("Загрузите карту")
            return
        disp = self.image.copy()
        # Draw anchor
        if self.zero_point:
            cv2.drawMarker(disp, self.zero_point, (0,255,0), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
            cv2.putText(disp, f"0", (self.zero_point[0]+10, self.zero_point[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
        # Draw ArUco
        for x, y, id in self.aruco_coords:
            cv2.circle(disp, (x, y), 10, (255,0,0), 2)
            cv2.putText(disp, str(id), (x+10, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
        # Draw points of interest
        for x, y in self.points_of_interest:
            cv2.circle(disp, (x, y), 7, (0,0,255), -1)
        # Draw coordinate axes if zero_point is set
        if self.zero_point:
            x0, y0 = self.zero_point
            length = 80
            # X axis
            dx = length if self.coord_system['x'] == 'right' else -length
            cv2.arrowedLine(disp, (x0, y0), (x0+dx, y0), (0,255,255), 2, tipLength=0.2)
            cv2.putText(disp, 'X', (x0+dx, y0-10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2)
            # Y axis
            dy = length if self.coord_system['y'] == 'down' else -length
            cv2.arrowedLine(disp, (x0, y0), (x0, y0+dy), (255,255,0), 2, tipLength=0.2)
            cv2.putText(disp, 'Y', (x0+10, y0+dy), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,0), 2)
        qimg = QImage(disp.data, disp.shape[1], disp.shape[0], disp.strides[0], QImage.Format_BGR888)
        pixmap = QPixmap.fromImage(qimg).scaled(self.label.size(), Qt.KeepAspectRatio)
        self.label.setPixmap(pixmap)

    def save_results(self):
        fname, _ = QFileDialog.getSaveFileName(self, "Сохранить результаты", "", "Text Files (*.txt)")
        if fname:
            with open(fname, "w") as f:
                f.write(f"Мировой ноль: {self.zero_point}\n")
                f.write(f"Система координат: X {self.coord_system['x']}, Y {self.coord_system['y']}\n")
                f.write(f"Привязка: {self.anchor_type}\n")
                f.write(f"Масштаб: {self.scale} м/пикс\n")
                f.write("ArUco:\n")
                for x, y, id in self.aruco_coords:
                    f.write(f"  id {id}: {x}, {y}\n")
                f.write("Точки интереса:\n")
                for x, y in self.points_of_interest:
                    f.write(f"  {x}, {y}\n")
            self.label.setText("Результаты сохранены.")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MapArucoUI()
    window.show()
    sys.exit(app.exec_())