#MAIN GUI

#importing important stuff to get things running
import PyQt6
from PyQt6.QtWidgets import QWidget, QLabel, QApplication, QMainWindow, QHBoxLayout, QLineEdit, QPushButton, QVBoxLayout, QFrame
from PyQt6.QtGui import QIcon, QPalette, QColor
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtCore import QUrl
from PyQt6.QtGui import QFont, QFontDatabase
import random
import time
import sys

#sets the mainwindow class upsss
class MainWindow(QMainWindow):
    #othersetup things
    def __init__(self):
        super().__init__()
        self.setupGUI()

    #main function to setup the main GUI

    #run something like this for all labels  
    def recieve_bno_data(self,dictionary):

        absolute_orientation_euler = dictionary["absolute-orientation-euler"]
        self.absolute_orientation_euler.setText("ABSOLUTE OREINTATION (EULER)\n" +  str(absolute_orientation_euler))
       
        absolute_orientation_quaterion = dictionary["absoulute-orientation-quaterion"]
        self.absolute_orientation_quaterion.setText("ABSOLUTE ORIENTATION (QUATERION)\n" + str(absolute_orientation_quaterion))

        angular_velocity = dictionary["angular-velocity"]
        self.angular_velocity.setText("ANGULAR VELOCITY\n" + str(angular_velocity))

        linear_velocity = dictionary["linear-velocity"]
        self.linear_velocity.setText("ANGULAR VELOCITY\n" + str(linear_velocity))

        magnetic_field_strength = dictionary["magnetic-field-strength"]
        self.magnetic_field_strength.setText("MAGNETIC FIELD STRENGTH\n" + str(magnetic_field_strength))

        linear_acceleration = dictionary["linear-acceleration"]
        self.linear_acceleration.setText("LINEAR ACCELERATION\n" + str(linear_acceleration))

        gravity = dictionary["gravity"]
        self.gravity.setText("GRAVITY\n" + str(gravity))

        temperature = dictionary["temperature"]
        self.temperature.setText("GRAVITY\n" + str(temperature))




    def setupGUI(self):
        #sets the size of the window and gives it a name
        self.resize(1200, 600)
        self.setWindowTitle("BLUE LOBSTER MAIN")

        #setting up font database
        font = QFont()
        font.setFamily('Helvetica')
        font.setPointSize(16)  # Set font size
        font.setBold(True)

        #sets up a bnodata widget
        self.bnoData = QWidget()
        self.bnoData.layout = QVBoxLayout()

        #sets up a bnodata widget for the right side
        self.bnoData2 = QWidget()
        self.bnoData2.layout = QVBoxLayout()

        #button frame
        font = QFont()
        font.setFamily('Helvetica')
        font.setPointSize(16)
        font.setBold(True)


        #repeat these 2 line formats to add to bnodata
        self.absolute_orientation_euler = QLabel("ABSOLUTE ORIENTATION (EULER):") 
        self.bnoData.layout.addWidget(self.absolute_orientation_euler)
        self.absolute_orientation_euler.setFont(font)
        self.absolute_orientation_euler.setStyleSheet(
        """
        QLabel {
            background-color: rgb(32,73,168);
            color: white;
            border-radius: 20px;
            border: 1px solid white;
        }

        """)
        

        self.absolute_orientation_quaterion = QLabel("ABSOLUTE ORIENTATION (QUATERION):")
        self.bnoData.layout.addWidget(self.absolute_orientation_quaterion)
        self.absolute_orientation_quaterion.setFont(font)
        self.absolute_orientation_quaterion.setStyleSheet(
        """
        QLabel {
            background-color: rgb(32,73,168);
            color: white;
            border-radius: 20px;
            border: 1px solid white;
        }

        """)

        self.angular_velocity = QLabel("ANGULAR VELOCITY:")
        self.bnoData.layout.addWidget(self.angular_velocity)
        self.angular_velocity.setFont(font)
        self.angular_velocity.setStyleSheet(
        """
        QLabel {
            background-color: rgb(32,73,168);
            color: white;
            border-radius: 20px;
            border: 1px solid white;
        }

        """)

        self.linear_velocity = QLabel("LINEAR VELOCITY:")
        self.bnoData.layout.addWidget(self.linear_velocity)
        self.linear_velocity.setFont(font)
        self.linear_velocity.setStyleSheet(
        """
        QLabel {
            background-color: rgb(32,73,168);
            color: white;
            border-radius: 20px;
            border: 1px solid white;
        }

        """)


        #adding to bnodata right
        self.magnetic_field_strength = QLabel("MAGNETIC FIELD STRENGTH:")
        self.bnoData2.layout.addWidget(self.magnetic_field_strength)
        self.magnetic_field_strength.setFont(font)
        self.magnetic_field_strength.setStyleSheet(
        """
        QLabel {
            background-color: rgb(32,73,168);
            color: white;
            border-radius: 20px;
            border: 1px solid white;
        }

        """)

        self.linear_acceleration = QLabel("LINEAR ACCELERATION:")
        self.bnoData2.layout.addWidget(self.linear_acceleration)
        self.linear_acceleration.setFont(font)
        self.linear_acceleration.setStyleSheet(
        """
        QLabel {
            background-color: rgb(32,73,168);
            color: white;
            border-radius: 20px;
            border: 1px solid white;
        }

        """)

        self.gravity = QLabel("GRAVITY:")
        self.bnoData2.layout.addWidget(self.gravity)
        self.gravity.setFont(font)
        self.gravity.setStyleSheet(
        """
        QLabel {
            background-color: rgb(32,73,168);
            color: white;
            border-radius: 20px;
            border: 1px solid white;
        }

        """)

        self.temperature = QLabel("TEMPERATURE:")
        self.bnoData2.layout.addWidget(self.temperature)
        self.temperature.setFont(font)
        self.temperature.setStyleSheet(
        """
        QLabel {
            background-color: rgb(32,73,168);
            color: white;
            border-radius: 20px;
            border: 1px solid white;
        }

        """)


        #once(this is for bnodata left)
        self.bnoData.setLayout(self.bnoData.layout)

        #once(this is for bnodata right)
        self.bnoData2.setLayout(self.bnoData2.layout)

        #adds a widget for a URL
        self.camData = QWebEngineView()
        self.camData.setUrl(QUrl("about:blank")) # change URL here. At the moment the URL is a blank square.
        self.camData.setLayout(QVBoxLayout())
        self.show()


        
        #changes background colour
        background = QPalette()
        #makes background blue
        background.setColor(QPalette.ColorRole.Window, QColor(32, 73, 168)) 
        self.setPalette(background)

        #adds buttons to the layout as widgets

        self.frame = QWidget()

        layout = QHBoxLayout()
        layout.addWidget(self.bnoData)
        layout.addWidget(self.camData)
        layout.addWidget(self.bnoData2)
    

        self.frame.setLayout(layout)
        self.setCentralWidget(self.frame)
        
        #Adds background boxes to bnodata
        '''bnoDataBox = QLabel()
        bnoDataBox.setFixedSize(100,100)
        bnoDataBox.setStyleSheet("background-color: rgb(102, 51, 102);")
        self.bnoData.layout.addWidget(bnoDataBox)'''


#some sorta setup thing. no clue what this does, but you need it for the program to run
if __name__ == "__main__":
    app = QApplication(sys.argv)

    form = MainWindow()
    form.show()

    #form.recieve_bno_data({"velocity" : random.randint(0,25)})

    app.exec()