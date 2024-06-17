
# GUI Code

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
from std_msgs.msg import Float64, Int32MultiArray
from PyQt5 import QtCore, QtGui, QtWidgets
from geometry_msgs.msg import Twist, Vector3
from tf_transformations import euler_from_quaternion
import sys
from .pilot_interface import init_pilot_interface
from multiprocessing import Process

class Ui_MainWindow(object):
    def __init__(self):

        self.deptharray = []
        self.timearray = []

        self.t1 = 0
        self.t2 = 0
        self.t3 = 0
        self.t4 = 0
        self.t5 = 0
        self.t6 = 0
        self.depth = 0
        self.temp = 0
        self.vx = 0
        self.wz = 0
        self.pressure = 0
        self.float_depth = 0
        self.time = 0
        self.values_array = []
        self.float_figs = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.start_flag = False

        for _ in range(100):
            mini_array = ["EX", self.pressure, self.float_depth, self.time]
            self.values_array.append(mini_array) 

        rclpy.init()
        self.node = rclpy.create_node("gui")
        self.create_subscriptionT = self.node.create_subscription(Int32MultiArray,'/ROV/thrusters', self.thrusters_callback,10)
        self.create_subscriptionD = self.node.create_subscription(Float64,'/ROV/depth', self.depth_callback, 10)
        self.create_subscriptionIMU = self.node.create_subscription(Imu,'/ROV/imu', self.imu_callback, 10)
        self.create_subscriptionVZ = self.node.create_subscription(Twist,'/ROV/cmd_vel', self.vel_callback, 10)
        self.create_subscriptionTEMP = self.node.create_subscription(Float64,'/ROV/temperature', self.temp_callback, 10)
        self.create_subscriptionF = self.node.create_subscription(Vector3,'/ROV/float', self.float_callback, 10)
        
    def thrusters_callback(self):
        self.t1 = self.create_subscriptionT.data[0]
        self.t2 = self.create_subscriptionT.data[1]
        self.t3 = self.create_subscriptionT.data[2]
        self.t4 = self.create_subscriptionT.data[3]
        self.t5 = self.create_subscriptionT.data[4]
        self.t6 = self.create_subscriptionT.data[5]
        self.update_UI()

    def depth_callback(self):
        self.depth = self.create_subscriptionD.data
        self.update_UI()

    def temp_callback(self):
        self.temp = self.create_subscriptionTEMP.data
        self.update_UI()

    def vel_callback(self):
        self.vx = self.create_subscriptionVZ.linear.x
        self.wz = self.create_subscriptionVZ.angular.z
        self.update_UI()
    
    def float_callback(self):
        if self.create_subscriptionF == [0,0,0]:
            plt.clear()
            self.values_array = []
            self.deptharray = []
            self.timearray = []
            self.start_flag = True

        elif self.create_subscriptionF != [10,10,10] and self.start_flag:
            self.pressure = self.create_subscriptionF.x
            self.float_depth = self.create_subscriptionF.y
            self.time = self.create_subscriptionF.z
            mini_array = ["EX", self.pressure, self.float_depth, self.time]
            self.values_array.append(mini_array) 
            self.deptharray.append(self.float_depth)
            self.timearray.append(self.time)

        elif self.create_subscriptionF == [10,10,10] and self.start_flag:
            for row, data in enumerate(self.values_array):
                for col, value in enumerate(data):
                    item = QtWidgets.QTableWidgetItem(value)
                    item.setTextAlignment(QtCore.Qt.AlignCenter) 
                    self.tableWidget.setItem(row, col, item)

            plt.plot(self.timearray, self.deptharray)
            plt.xlabel("Time")
            plt.ylabel("Depth")
        
            plt.savefig(f'float_fig({self.float_figs}).png')
            #plt.show()
        
            self.label_2.setPixmap(QtGui.QPixmap(f"float_fig({self.float_figs}).png"))
            self.float_figs += 1
            self.start_flag = False

        self.update_UI()

    def imu_callback(self):
        orientation_q = self.create_subscriptionIMU.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)
        self.yaw *= -1
        self.pitch *= -1
        self.update_UI()
  
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.showFullScreen()

        MainWindow.setMaximumSize(QtCore.QSize(14000000, 14000000))
        # background image.
        MainWindow.setStyleSheet("QMainWindow{border-image: url('/home/atef/ROV 2024/colcon_ws/src/rov_24/rov_24/guiorange.jpeg'); \
                                 background-repeat: no-repeat;}")


        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.centralwidget.setGeometry(QtCore.QRect(0, 0, 1920, 1080))
        self.textBrowser_3 = QtWidgets.QTextBrowser(self.centralwidget)
        self.textBrowser_3.setGeometry(QtCore.QRect(650, 220, 81, 41))
        self.textBrowser_3.setObjectName("textBrowser_3")
        #yaw  = "0000"
        self.textBrowser_3.setText(str(self.yaw))


        self.pitch_widget = QtWidgets.QLabel(self.centralwidget)
        self.pitch_widget.setGeometry(QtCore.QRect(550, 170, 67, 21))
        font = QtGui.QFont()
        font.setFamily("Ubuntu Mono")
        font.setPointSize(15)
        font.setBold(True)
        font.setWeight(75)
        self.pitch_widget.setFont(font)
        self.pitch_widget.setStyleSheet("color: rgb(255, 255, 255);")
        self.pitch_widget.setAlignment(QtCore.Qt.AlignCenter)
        self.pitch_widget.setObjectName("pitch")
        self.textBrowser = QtWidgets.QTextBrowser(self.centralwidget)
        self.textBrowser.setGeometry(QtCore.QRect(300, 440, 81, 41))
        self.textBrowser.setReadOnly(True)
        self.textBrowser.setObjectName("textBrowser")
        #depth  = "0000"
        self.textBrowser.setText(str(self.depth))


        self.vx_widget = QtWidgets.QLabel(self.centralwidget)
        self.vx_widget.setGeometry(QtCore.QRect(550, 280, 67, 21))
        font = QtGui.QFont()
        font.setFamily("Ubuntu Mono")
        font.setPointSize(15)
        font.setBold(True)
        font.setWeight(75)
        self.vx_widget.setFont(font)
        self.vx_widget.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.vx_widget.setStyleSheet("color: rgb(255, 255, 255);")
        self.vx_widget.setAlignment(QtCore.Qt.AlignCenter)
        self.vx_widget.setObjectName("vx")
        self.modeling = QtWidgets.QPushButton(self.centralwidget)
        self.modeling.setGeometry(QtCore.QRect(580, 440, 111, 41))
        font = QtGui.QFont()
        font.setFamily("Ubuntu Mono")
        font.setPointSize(15)
        font.setBold(True)
        font.setWeight(75)
        self.modeling.setFont(font)
        self.modeling.setStyleSheet("color: rgb(0, 0, 0);")
        self.modeling.setObjectName("modeling")
        self.modeling.clicked.connect(self._3D_clicked)
        self.height = QtWidgets.QLabel(self.centralwidget)
        self.height.setGeometry(QtCore.QRect(190, 500, 71, 31))
        font = QtGui.QFont()
        font.setFamily("Ubuntu Mono")
        font.setPointSize(15)
        font.setBold(True)
        font.setWeight(75)
        self.height.setFont(font)
        self.height.setStyleSheet("color: rgb(255, 255, 255);")
        self.height.setAlignment(QtCore.Qt.AlignCenter)
        self.height.setObjectName("height")
        self.wz_widget = QtWidgets.QLabel(self.centralwidget)
        self.wz_widget.setGeometry(QtCore.QRect(550, 340, 67, 21))
        font = QtGui.QFont()
        font.setFamily("Ubuntu Mono")
        font.setPointSize(15)
        font.setBold(True)
        font.setWeight(75)
        self.wz_widget.setFont(font)
        self.wz_widget.setStyleSheet("color: rgb(255, 255, 255);")
        self.wz_widget.setAlignment(QtCore.Qt.AlignCenter)
        self.wz_widget.setObjectName("wz")
        self.textBrowser_5 = QtWidgets.QTextBrowser(self.centralwidget)
        self.textBrowser_5.setGeometry(QtCore.QRect(650, 100, 81, 41))
        self.textBrowser_5.setObjectName("textBrowser_5")
        #roll = "0000"
        self.textBrowser_5.setText(str(self.roll))


        self.yaw_widget = QtWidgets.QLabel(self.centralwidget)
        self.yaw_widget.setGeometry(QtCore.QRect(550, 230, 67, 21))
        font = QtGui.QFont()
        font.setFamily("Ubuntu Mono")
        font.setPointSize(15)
        font.setBold(True)
        font.setWeight(75)
        self.yaw_widget.setFont(font)
        self.yaw_widget.setStyleSheet("color: rgb(255, 255, 255);")
        self.yaw_widget.setAlignment(QtCore.Qt.AlignCenter)
        self.yaw_widget.setObjectName("yaw")
        self.textBrowser_7 = QtWidgets.QTextBrowser(self.centralwidget)
        self.textBrowser_7.setGeometry(QtCore.QRect(300, 500, 81, 41))
        self.textBrowser_7.setObjectName("textBrowser_7")
        #temp  = "0000"
        self.textBrowser_7.setText(str(self.temp))



        self.depth_widget = QtWidgets.QLabel(self.centralwidget)
        self.depth_widget.setGeometry(QtCore.QRect(190, 440, 71, 31))
        font = QtGui.QFont()
        font.setFamily("Ubuntu Mono")
        font.setPointSize(15)
        font.setBold(True)
        font.setWeight(75)
        self.depth_widget.setFont(font)
        self.depth_widget.setStyleSheet("color: rgb(255, 255, 255);")
        self.depth_widget.setAlignment(QtCore.Qt.AlignCenter)
        self.depth_widget.setObjectName("depth")
        self.circle = QtWidgets.QPushButton(self.centralwidget)
        self.circle.setGeometry(QtCore.QRect(580, 500, 111, 41))

        font = QtGui.QFont()
        font.setFamily("Ubuntu Mono")
        font.setPointSize(15)
        font.setBold(True)
        font.setWeight(75)
        self.circle.setFont(font)
        self.circle.setStyleSheet("color: rgb(0, 0, 0);")
        self.circle.setObjectName("circle")
        self.circle.clicked.connect(self.circle_clicked)

        self.textBrowser_4 = QtWidgets.QTextBrowser(self.centralwidget)
        self.textBrowser_4.setGeometry(QtCore.QRect(650, 160, 81, 41))
        self.textBrowser_4.setObjectName("textBrowser_4")
        #pitch = "0000"
        self.textBrowser_4.setText(str(self.pitch))

        self.t2_widget = QtWidgets.QTextBrowser(self.centralwidget)
        self.t2_widget.setGeometry(QtCore.QRect(370, 120, 71, 31))
        self.t2_widget.setObjectName("t2")
        #t2 = "0000"
        self.t2_widget.setText(str(self.t2))

        self.textBrowser_6 = QtWidgets.QTextBrowser(self.centralwidget)
        self.textBrowser_6.setGeometry(QtCore.QRect(650, 340, 81, 41))
        self.textBrowser_6.setObjectName("textBrowser_6")
        #wz  = "0000"
        self.textBrowser_6.setText(str(self.wz))

        self.textBrowser_2 = QtWidgets.QTextBrowser(self.centralwidget)
        self.textBrowser_2.setGeometry(QtCore.QRect(650, 280, 81, 41))
        self.textBrowser_2.setObjectName("textBrowser_2")
        #vx  = "0000"
        self.textBrowser_2.setText(str(self.vx))

        self.t3_widget = QtWidgets.QTextBrowser(self.centralwidget)
        self.t3_widget.setGeometry(QtCore.QRect(140, 190, 71, 31))
        self.t3_widget.setObjectName("t3")
        #t3 = "0000"
        self.t3_widget.setText(str(self.t3))


        self.roll_widget = QtWidgets.QLabel(self.centralwidget)
        self.roll_widget.setGeometry(QtCore.QRect(550, 110, 67, 21))
        font = QtGui.QFont()
        font.setFamily("Ubuntu Mono")
        font.setPointSize(15)
        font.setBold(True)
        font.setWeight(75)
        self.roll_widget.setFont(font)
        self.roll_widget.setStyleSheet("color: rgb(255, 255, 255);")
        self.roll_widget.setAlignment(QtCore.Qt.AlignCenter)
        self.roll_widget.setObjectName("roll")

        self.t6_widget = QtWidgets.QTextBrowser(self.centralwidget)
        self.t6_widget.setGeometry(QtCore.QRect(140, 310, 71, 31))
        self.t6_widget.setObjectName("t6")
        #t6 = "0000"
        self.t6_widget.setText(str(self.t6))

        self.t1_widget = QtWidgets.QTextBrowser(self.centralwidget)
        self.t1_widget.setGeometry(QtCore.QRect(140, 120, 71, 31))
        self.t1_widget.setObjectName("t1")
        #t1 = "0000"
        self.t1_widget.setText(str(self.t1))

        self.t5_widget = QtWidgets.QTextBrowser(self.centralwidget)
        self.t5_widget.setGeometry(QtCore.QRect(370, 310, 71, 31))
        self.t5_widget.setObjectName("t5")
        #t5 = "0000"
        self.t5_widget.setText(str(self.t5))

        self.t4_widget = QtWidgets.QTextBrowser(self.centralwidget)
        self.t4_widget.setGeometry(QtCore.QRect(370, 190, 71, 31))
        self.t4_widget.setObjectName("t4")
        #t4 = "0000"
        self.t4_widget.setText(str(self.t4))

        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(480, 30, 411, 41))
        self.label.setAutoFillBackground(False)
        self.label.setStyleSheet("color: rgb(255, 255, 255);")
        self.label.setObjectName("label")

# ------------------------------------------
# Plotting the graph:

        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_2.setGeometry(QtCore.QRect(860, 380, 321, 201))
        self.label_2.setStyleSheet("QMainWindow{border-image: url('/home/atef/ROV 2024/colcon_ws/src/rov_24/rov_24/gra.jpeg'); \
                                 background-repeat: no-repeat;}")
        self.label_2.setText("")

# Put the same path below

        self.label_2.setPixmap(QtGui.QPixmap("/home/atef/ROV 2024/colcon_ws/src/rov_24/rov_24/gra.jpeg"))
        self.label_2.setScaledContents(True)
        self.label_2.setObjectName("label_2")

# ------------------------------------------

        self.tableWidget = QtWidgets.QTableWidget(self.centralwidget)
        self.tableWidget.setGeometry(QtCore.QRect(800, 90, 441, 261))
        font = QtGui.QFont()
        font.setFamily("Ubuntu Mono")
        font.setPointSize(15)
        font.setBold(True)
        font.setWeight(75)
        self.tableWidget.setFont(font)
        self.tableWidget.setAutoFillBackground(True)
        self.tableWidget.setObjectName("tableWidget")
        self.tableWidget.setColumnCount(4)
        self.tableWidget.setRowCount(2000)



# Populate the table with values from the array

        for row, data in enumerate(self.values_array):
            for col, value in enumerate(data):
               item = QtWidgets.QTableWidgetItem(value)
               item.setTextAlignment(QtCore.Qt.AlignCenter) 
               self.tableWidget.setItem(row, col, item)

        for i in range(2000):
            item = QtWidgets.QTableWidgetItem()
            self.tableWidget.setVerticalHeaderItem(i, item)
        
        for i in range(4):
            item = QtWidgets.QTableWidgetItem()
            self.tableWidget.setHorizontalHeaderItem(i, item)

        self.label_3 = QtWidgets.QLabel(self.centralwidget)
        self.label_3.setGeometry(QtCore.QRect(130, 100, 321, 281))
        self.label_3.setAutoFillBackground(False)

# ROV image should be on the user's laptop

        self.label_3.setStyleSheet("QMainWindow{border-image: url('/home/atef/ROV 2024/colcon_ws/src/rov_24/rov_24/rov.jpeg'); \
                                 background-repeat: no-repeat;}")
        self.label_3.setText("")
        self.label_3.setPixmap(QtGui.QPixmap("/home/atef/ROV 2024/colcon_ws/src/rov_24/rov_24/rov.jpeg"))
        
        self.label_3.setScaledContents(True)
        self.label_3.setObjectName("label_3")
        self.label_3.raise_()
        self.textBrowser_3.raise_()
        self.pitch_widget.raise_()
        self.textBrowser.raise_()
        self.vx_widget.raise_()
        self.modeling.raise_()
        self.height.raise_()
        self.wz_widget.raise_()
        self.textBrowser_5.raise_()
        self.yaw_widget.raise_()
        self.textBrowser_7.raise_()
        self.depth_widget.raise_()
        self.circle.raise_()
        self.textBrowser_4.raise_()
        self.t2_widget.raise_()
        self.textBrowser_6.raise_()
        self.textBrowser_2.raise_()
        self.t3_widget.raise_()
        self.roll_widget.raise_()
        self.t6_widget.raise_()
        self.t1_widget.raise_()
        self.t5_widget.raise_()
        self.t4_widget.raise_()
        self.label.raise_()
        self.label_2.raise_()
        self.tableWidget.raise_()
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1389, 21))
        self.menubar.setObjectName("menubar")
        self.menuCO_PILOT = QtWidgets.QMenu(self.menubar)
        self.menuCO_PILOT.setObjectName("menuCO_PILOT")
        MainWindow.setMenuBar(self.menubar)
        self.menubar.addAction(self.menuCO_PILOT.menuAction())

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def update_UI(self):
        self.textBrowser_3.setText(str(self.yaw))
        self.textBrowser.setText(str(self.depth))
        self.textBrowser_5.setText(str(self.roll))
        self.textBrowser_4.setText(str(self.pitch))
        self.textBrowser_7.setText(str(self.temp))
        self.textBrowser_2.setText(str(self.vx))
        self.textBrowser_6.setText(str(self.wz))
        self.t1_widget.setText(str(self.t1))
        self.t2_widget.setText(str(self.t2))
        self.t3_widget.setText(str(self.t3))
        self.t4_widget.setText(str(self.t4))
        self.t5_widget.setText(str(self.t5))
        self.t6_widget.setText(str(self.t6))

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.pitch_widget.setText(_translate("MainWindow", "Pitch"))

        self.vx_widget.setText(_translate("MainWindow", "Vx"))
        self.modeling.setText(_translate("MainWindow", "3D"))
        self.height.setText(_translate("MainWindow", "Temp"))
        self.wz_widget.setText(_translate("MainWindow", "Wz"))

        self.yaw_widget.setText(_translate("MainWindow", "Yaw"))
        self.depth_widget.setText(_translate("MainWindow", "Depth"))
        self.circle.setText(_translate("MainWindow", "Circle"))

        self.roll_widget.setText(_translate("MainWindow", "Roll"))
        self.label.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-size:20pt; font-weight:600;\">Shiro Kaijin Control Room</span></p></body></html>"))
        for i in range(2000):
            item = self.tableWidget.verticalHeaderItem(i)
            item.setText(_translate("MainWindow", str(i)))
        item = self.tableWidget.horizontalHeaderItem(0)
        item.setText(_translate("MainWindow", "Company Number"))
        item = self.tableWidget.horizontalHeaderItem(1)
        item.setText(_translate("MainWindow", "Pressure"))
        item = self.tableWidget.horizontalHeaderItem(2)
        item.setText(_translate("MainWindow", "Depth"))
        item = self.tableWidget.horizontalHeaderItem(3)
        item.setText(_translate("MainWindow", "Time"))
        self.menuCO_PILOT.setTitle(_translate("MainWindow", "CO_PILOT"))

    def circle_clicked(self):
        pass

    def _3D_clicked(self):
        pass

def init_copilot_interface():
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

def main():
    p1 = Process(target=init_pilot_interface)
    p2 = Process(target=init_copilot_interface)
    p1.start()
    p2.start()

if __name__ == "__main__":
    main()    