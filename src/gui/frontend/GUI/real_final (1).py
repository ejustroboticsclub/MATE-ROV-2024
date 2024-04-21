
# GUI Code

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
from std_msgs.msg import Float64
from PyQt5 import QtCore, QtGui, QtWidgets
from geometry_msgs.msg import Twist, Vector3
from tf_transformations import euler_from_quaternion


class Ui_MainWindow(object):
    def _init_(self):

        self.deptharray = []
        self.timearray = []


        self.node = rclpy.create_node("gui")
        self.create_subscriptionT = self.node.create_subscription(Twist,'/ROV/thrusters', self.setupUi,10)
        self.create_subscriptionD = self.node.create_subscription(Float64,'/ROV/depth', self.setupUi, 10)
        self.create_subscriptionIMU = self.node.create_subscription(Imu,'/ROV/imu', self.setupUi, 10)
        self.create_subscriptionVZ = self.node.create_subscription(Twist,'/ROV/cmd_vel', self.setupUi, 10)
        self.create_subscriptionTEMP = self.node.create_subscription(Float64,'/ROV/temperature', self.setupUi, 10)
        self.create_subscriptionF = self.node.create_subscription(Vector3,'/ROV/float', self.setupUi, 10)


    def setupUi(self, MainWindow):

        t1 = self.create_subscriptionT.linear.x
        t2 = self.create_subscriptionT.linear.y
        t3 = self.create_subscriptionT.linear.z
        t4 = self.create_subscriptionT.angular.x
        t5 = self.create_subscriptionT.angular.y
        t6 = self.create_subscriptionT.angular.z

        depth = self.create_subscriptionD.data
        temp = self.create_subscriptionTEMP.data

        vx = self.create_subscriptionVZ.linear.x
        wz = self.create_subscriptionVZ.angular.z

        pressure = self.create_subscriptionF.x
        Depth = self.create_subscriptionF.y
        time = self.create_subscriptionF.z

        for i in range(100):
            mini_array = ["0", pressure, Depth, time]
            values_array = []
            values_array.append(mini_array) 
            

        orientation_q = self.create_subscriptionIMU.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)


        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1389, 647)
        MainWindow.setMaximumSize(QtCore.QSize(14000000, 14000000))
        # background image.
        MainWindow.setStyleSheet("QMainWindow{border-image: url(:/images/guiorange.jpeg);}")


        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.textBrowser_3 = QtWidgets.QTextBrowser(self.centralwidget)
        self.textBrowser_3.setGeometry(QtCore.QRect(650, 220, 81, 41))
        self.textBrowser_3.setObjectName("textBrowser_3")
        #yaw  = "0000"
        self.textBrowser_3.setText(yaw)


        self.pitch = QtWidgets.QLabel(self.centralwidget)
        self.pitch.setGeometry(QtCore.QRect(550, 170, 67, 21))
        font = QtGui.QFont()
        font.setFamily("Ubuntu Mono")
        font.setPointSize(15)
        font.setBold(True)
        font.setWeight(75)
        self.pitch.setFont(font)
        self.pitch.setStyleSheet("color: rgb(255, 255, 255);")
        self.pitch.setAlignment(QtCore.Qt.AlignCenter)
        self.pitch.setObjectName("pitch")
        self.textBrowser = QtWidgets.QTextBrowser(self.centralwidget)
        self.textBrowser.setGeometry(QtCore.QRect(300, 440, 81, 41))
        self.textBrowser.setReadOnly(True)
        self.textBrowser.setObjectName("textBrowser")
        #depth  = "0000"
        self.textBrowser.setText(depth)


        self.vx = QtWidgets.QLabel(self.centralwidget)
        self.vx.setGeometry(QtCore.QRect(550, 280, 67, 21))
        font = QtGui.QFont()
        font.setFamily("Ubuntu Mono")
        font.setPointSize(15)
        font.setBold(True)
        font.setWeight(75)
        self.vx.setFont(font)
        self.vx.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.vx.setStyleSheet("color: rgb(255, 255, 255);")
        self.vx.setAlignment(QtCore.Qt.AlignCenter)
        self.vx.setObjectName("vx")
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
        self.wz = QtWidgets.QLabel(self.centralwidget)
        self.wz.setGeometry(QtCore.QRect(550, 340, 67, 21))
        font = QtGui.QFont()
        font.setFamily("Ubuntu Mono")
        font.setPointSize(15)
        font.setBold(True)
        font.setWeight(75)
        self.wz.setFont(font)
        self.wz.setStyleSheet("color: rgb(255, 255, 255);")
        self.wz.setAlignment(QtCore.Qt.AlignCenter)
        self.wz.setObjectName("wz")
        self.textBrowser_5 = QtWidgets.QTextBrowser(self.centralwidget)
        self.textBrowser_5.setGeometry(QtCore.QRect(650, 100, 81, 41))
        self.textBrowser_5.setObjectName("textBrowser_5")
        #roll = "0000"
        self.textBrowser_5.setText(roll)


        self.yaw = QtWidgets.QLabel(self.centralwidget)
        self.yaw.setGeometry(QtCore.QRect(550, 230, 67, 21))
        font = QtGui.QFont()
        font.setFamily("Ubuntu Mono")
        font.setPointSize(15)
        font.setBold(True)
        font.setWeight(75)
        self.yaw.setFont(font)
        self.yaw.setStyleSheet("color: rgb(255, 255, 255);")
        self.yaw.setAlignment(QtCore.Qt.AlignCenter)
        self.yaw.setObjectName("yaw")
        self.textBrowser_7 = QtWidgets.QTextBrowser(self.centralwidget)
        self.textBrowser_7.setGeometry(QtCore.QRect(300, 500, 81, 41))
        self.textBrowser_7.setObjectName("textBrowser_7")
        #temp  = "0000"
        self.textBrowser_7.setText(temp)



        self.depth = QtWidgets.QLabel(self.centralwidget)
        self.depth.setGeometry(QtCore.QRect(190, 440, 71, 31))
        font = QtGui.QFont()
        font.setFamily("Ubuntu Mono")
        font.setPointSize(15)
        font.setBold(True)
        font.setWeight(75)
        self.depth.setFont(font)
        self.depth.setStyleSheet("color: rgb(255, 255, 255);")
        self.depth.setAlignment(QtCore.Qt.AlignCenter)
        self.depth.setObjectName("depth")
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

        self.textBrowser_4 = QtWidgets.QTextBrowser(self.centralwidget)
        self.textBrowser_4.setGeometry(QtCore.QRect(650, 160, 81, 41))
        self.textBrowser_4.setObjectName("textBrowser_4")
        #pitch = "0000"
        self.textBrowser_4.setText(pitch)


        self.t2 = QtWidgets.QTextBrowser(self.centralwidget)
        self.t2.setGeometry(QtCore.QRect(370, 120, 71, 31))
        self.t2.setObjectName("t2")
        #t2 = "0000"
        self.t2.setText(str(t2))


        self.textBrowser_6 = QtWidgets.QTextBrowser(self.centralwidget)
        self.textBrowser_6.setGeometry(QtCore.QRect(650, 340, 81, 41))
        self.textBrowser_6.setObjectName("textBrowser_6")
        #wz  = "0000"
        self.textBrowser_6.setText(wz)


        self.textBrowser_2 = QtWidgets.QTextBrowser(self.centralwidget)
        self.textBrowser_2.setGeometry(QtCore.QRect(650, 280, 81, 41))
        self.textBrowser_2.setObjectName("textBrowser_2")
        #vx  = "0000"
        self.textBrowser_2.setText(vx)

        self.t3 = QtWidgets.QTextBrowser(self.centralwidget)
        self.t3.setGeometry(QtCore.QRect(140, 190, 71, 31))
        self.t3.setObjectName("t3")
        #t3 = "0000"
        self.t3.setText(str(t3))


        self.roll = QtWidgets.QLabel(self.centralwidget)
        self.roll.setGeometry(QtCore.QRect(550, 110, 67, 21))
        font = QtGui.QFont()
        font.setFamily("Ubuntu Mono")
        font.setPointSize(15)
        font.setBold(True)
        font.setWeight(75)
        self.roll.setFont(font)
        self.roll.setStyleSheet("color: rgb(255, 255, 255);")
        self.roll.setAlignment(QtCore.Qt.AlignCenter)
        self.roll.setObjectName("roll")

        self.t6 = QtWidgets.QTextBrowser(self.centralwidget)
        self.t6.setGeometry(QtCore.QRect(140, 310, 71, 31))
        self.t6.setObjectName("t6")
        #t6 = "0000"
        self.t6.setText(str(t6))

        self.t1 = QtWidgets.QTextBrowser(self.centralwidget)
        self.t1.setGeometry(QtCore.QRect(140, 120, 71, 31))
        self.t1.setObjectName("t1")
        #t1 = "0000"
        self.t1.setText(str(t1))

        self.t5 = QtWidgets.QTextBrowser(self.centralwidget)
        self.t5.setGeometry(QtCore.QRect(370, 310, 71, 31))
        self.t5.setObjectName("t5")
        #t5 = "0000"
        self.t5.setText(str(t5))

        self.t4 = QtWidgets.QTextBrowser(self.centralwidget)
        self.t4.setGeometry(QtCore.QRect(370, 190, 71, 31))
        self.t4.setObjectName("t4")
        #t4 = "0000"
        self.t4.setText(str(t4))

        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(480, 30, 411, 41))
        self.label.setAutoFillBackground(False)
        self.label.setStyleSheet("color: rgb(255, 255, 255);")
        self.label.setObjectName("label")

# ------------------------------------------
# Plotting the graph:

        if self.create_subscriptionF == [0,0,0]:
            plt.clear()
            while self.create_subscriptionF != [10,10,10]:
                self.deptharray.append(Depth)
                self.timearray.append(time)
                pressure = self.create_subscriptionF.x
                Depth = self.create_subscriptionF.y
                time = self.create_subscriptionF.z

            plt.plot(time,Depth)
            plt.xlabel("Time")
            plt.ylabel("Depth")
        
# User decides the path.

            plt.savefig('path/name')
            #plt.show()

        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_2.setGeometry(QtCore.QRect(860, 380, 321, 201))
        self.label_2.setStyleSheet("image: url(:/pictures/gra.jpeg);")
        self.label_2.setText("")

# Put the same path below

        self.label_2.setPixmap(QtGui.QPixmap("E:\College\Robotics\ROV\gra.jpeg"))
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

        for row, data in enumerate(values_array):
            for col, value in enumerate(data):
               item = QtWidgets.QTableWidgetItem(value)
               item.setTextAlignment(QtCore.Qt.AlignCenter) 
               self.tableWidget.setItem(row, col, item)


        item = QtWidgets.QTableWidgetItem()
        self.tableWidget.setVerticalHeaderItem(0, item)
        item = QtWidgets.QTableWidgetItem()
        self.tableWidget.setVerticalHeaderItem(1, item)
        item = QtWidgets.QTableWidgetItem()
        self.tableWidget.setVerticalHeaderItem(2, item)
        item = QtWidgets.QTableWidgetItem()
        self.tableWidget.setVerticalHeaderItem(3, item)
        item = QtWidgets.QTableWidgetItem()
        self.tableWidget.setVerticalHeaderItem(4, item)
        item = QtWidgets.QTableWidgetItem()
        self.tableWidget.setVerticalHeaderItem(5, item)
        item = QtWidgets.QTableWidgetItem()
        self.tableWidget.setVerticalHeaderItem(6, item)
        item = QtWidgets.QTableWidgetItem()
        self.tableWidget.setVerticalHeaderItem(7, item)
        item = QtWidgets.QTableWidgetItem()
        self.tableWidget.setVerticalHeaderItem(8, item)
        item = QtWidgets.QTableWidgetItem()
        self.tableWidget.setVerticalHeaderItem(9, item)
        item = QtWidgets.QTableWidgetItem()
        self.tableWidget.setVerticalHeaderItem(10, item)
        item = QtWidgets.QTableWidgetItem()
        self.tableWidget.setVerticalHeaderItem(11, item)
        item = QtWidgets.QTableWidgetItem()
        self.tableWidget.setVerticalHeaderItem(12, item)
        item = QtWidgets.QTableWidgetItem()
        self.tableWidget.setVerticalHeaderItem(13, item)
        item = QtWidgets.QTableWidgetItem()
        self.tableWidget.setVerticalHeaderItem(14, item)
        item = QtWidgets.QTableWidgetItem()
        self.tableWidget.setVerticalHeaderItem(15, item)
        item = QtWidgets.QTableWidgetItem()
        self.tableWidget.setHorizontalHeaderItem(0, item)
        item = QtWidgets.QTableWidgetItem()
        self.tableWidget.setHorizontalHeaderItem(1, item)
        item = QtWidgets.QTableWidgetItem()
        self.tableWidget.setHorizontalHeaderItem(2, item)
        item = QtWidgets.QTableWidgetItem()
        self.tableWidget.setHorizontalHeaderItem(3, item)
        self.label_3 = QtWidgets.QLabel(self.centralwidget)
        self.label_3.setGeometry(QtCore.QRect(130, 100, 321, 281))
        self.label_3.setAutoFillBackground(False)

# ROV image should be on the user's laptop

        self.label_3.setStyleSheet("border-image: url(:/pictures/rov.jpeg);")
        self.label_3.setText("")
        self.label_3.setPixmap(QtGui.QPixmap("E:\College\Robotics\ROV\ rov.jpeg"))
        
        self.label_3.setScaledContents(True)
        self.label_3.setObjectName("label_3")
        self.label_3.raise_()
        self.textBrowser_3.raise_()
        self.pitch.raise_()
        self.textBrowser.raise_()
        self.vx.raise_()
        self.modeling.raise_()
        self.height.raise_()
        self.wz.raise_()
        self.textBrowser_5.raise_()
        self.yaw.raise_()
        self.textBrowser_7.raise_()
        self.depth.raise_()
        self.circle.raise_()
        self.textBrowser_4.raise_()
        self.t2.raise_()
        self.textBrowser_6.raise_()
        self.textBrowser_2.raise_()
        self.t3.raise_()
        self.roll.raise_()
        self.t6.raise_()
        self.t1.raise_()
        self.t5.raise_()
        self.t4.raise_()
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

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.pitch.setText(_translate("MainWindow", "Pitch"))

        
        self.vx.setText(_translate("MainWindow", "Vx"))
        self.modeling.setText(_translate("MainWindow", "3D"))
        self.height.setText(_translate("MainWindow", "Temp"))
        self.wz.setText(_translate("MainWindow", "Wz"))

        
        self.yaw.setText(_translate("MainWindow", "Yaw"))
        self.depth.setText(_translate("MainWindow", "Depth"))
        self.circle.setText(_translate("MainWindow", "Circle"))


        self.roll.setText(_translate("MainWindow", "Roll"))
        self.label.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-size:20pt; font-weight:600;\">Shiro Kaijin Control Room</span></p></body></html>"))
        item = self.tableWidget.verticalHeaderItem(0)
        item.setText(_translate("MainWindow", "1"))
        item = self.tableWidget.verticalHeaderItem(1)
        item.setText(_translate("MainWindow", "2"))
        item = self.tableWidget.verticalHeaderItem(2)
        item.setText(_translate("MainWindow", "3"))
        item = self.tableWidget.verticalHeaderItem(3)
        item.setText(_translate("MainWindow", "4"))
        item = self.tableWidget.verticalHeaderItem(4)
        item.setText(_translate("MainWindow", "5"))
        item = self.tableWidget.verticalHeaderItem(5)
        item.setText(_translate("MainWindow", "6"))
        item = self.tableWidget.verticalHeaderItem(6)
        item.setText(_translate("MainWindow", "7"))
        item = self.tableWidget.verticalHeaderItem(7)
        item.setText(_translate("MainWindow", "8"))
        item = self.tableWidget.verticalHeaderItem(8)
        item.setText(_translate("MainWindow", "9"))
        item = self.tableWidget.verticalHeaderItem(9)
        item.setText(_translate("MainWindow", "10"))
        item = self.tableWidget.verticalHeaderItem(10)
        item.setText(_translate("MainWindow", "11"))
        item = self.tableWidget.verticalHeaderItem(11)
        item.setText(_translate("MainWindow", "12"))
        item = self.tableWidget.verticalHeaderItem(12)
        item.setText(_translate("MainWindow", "13"))
        item = self.tableWidget.verticalHeaderItem(13)
        item.setText(_translate("MainWindow", "15"))
        item = self.tableWidget.verticalHeaderItem(14)
        item.setText(_translate("MainWindow", "16"))
        item = self.tableWidget.verticalHeaderItem(15)
        item.setText(_translate("MainWindow", "17"))
        item = self.tableWidget.horizontalHeaderItem(0)
        item.setText(_translate("MainWindow", "Company Number"))
        item = self.tableWidget.horizontalHeaderItem(1)
        item.setText(_translate("MainWindow", "Pressure"))
        item = self.tableWidget.horizontalHeaderItem(2)
        item.setText(_translate("MainWindow", "Depth"))
        item = self.tableWidget.horizontalHeaderItem(3)
        item.setText(_translate("MainWindow", "Time"))
        self.menuCO_PILOT.setTitle(_translate("MainWindow", "CO_PILOT"))
import pictures_rc


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
