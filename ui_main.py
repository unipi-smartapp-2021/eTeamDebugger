# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'main.ui'
##
## Created by: Qt User Interface Compiler version 6.2.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QAction, QBrush, QColor, QConicalGradient,
    QCursor, QFont, QFontDatabase, QGradient,
    QIcon, QImage, QKeySequence, QLinearGradient,
    QPainter, QPalette, QPixmap, QRadialGradient,
    QTransform)
from PySide6.QtWidgets import (QApplication, QCheckBox, QLabel, QListWidget,
    QListWidgetItem, QMainWindow, QMenu, QMenuBar,
    QPushButton, QSizePolicy, QWidget)

class Ui_eTeamDebugger(object):
    def setupUi(self, eTeamDebugger):
        if not eTeamDebugger.objectName():
            eTeamDebugger.setObjectName(u"eTeamDebugger")
        eTeamDebugger.resize(662, 588)
        self.importtopic = QAction(eTeamDebugger)
        self.importtopic.setObjectName(u"importtopic")
        self.exporttopic = QAction(eTeamDebugger)
        self.exporttopic.setObjectName(u"exporttopic")
        self.importsnapshots = QAction(eTeamDebugger)
        self.importsnapshots.setObjectName(u"importsnapshots")
        self.exportsnapshots = QAction(eTeamDebugger)
        self.exportsnapshots.setObjectName(u"exportsnapshots")
        self.centralwidget = QWidget(eTeamDebugger)
        self.centralwidget.setObjectName(u"centralwidget")
        self.topiclist = QListWidget(self.centralwidget)
        self.topiclist.setObjectName(u"topiclist")
        self.topiclist.setGeometry(QRect(0, 30, 321, 251))
        self.snaplist = QListWidget(self.centralwidget)
        self.snaplist.setObjectName(u"snaplist")
        self.snaplist.setGeometry(QRect(320, 30, 341, 491))
        self.label = QLabel(self.centralwidget)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(0, 10, 91, 16))
        self.label_2 = QLabel(self.centralwidget)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setGeometry(QRect(320, 10, 111, 16))
        self.addtopic = QPushButton(self.centralwidget)
        self.addtopic.setObjectName(u"addtopic")
        self.addtopic.setGeometry(QRect(0, 530, 81, 28))
        self.addtopic.setAutoDefault(False)
        self.removetopic = QPushButton(self.centralwidget)
        self.removetopic.setObjectName(u"removetopic")
        self.removetopic.setGeometry(QRect(80, 530, 111, 28))
        self.newsnap = QPushButton(self.centralwidget)
        self.newsnap.setObjectName(u"newsnap")
        self.newsnap.setGeometry(QRect(320, 530, 71, 28))
        self.removesnap = QPushButton(self.centralwidget)
        self.removesnap.setObjectName(u"removesnap")
        self.removesnap.setGeometry(QRect(390, 530, 71, 28))
        self.playsnap = QPushButton(self.centralwidget)
        self.playsnap.setObjectName(u"playsnap")
        self.playsnap.setGeometry(QRect(490, 530, 71, 28))
        self.openrviz = QCheckBox(self.centralwidget)
        self.openrviz.setObjectName(u"openrviz")
        self.openrviz.setGeometry(QRect(570, 530, 87, 31))
        self.openrviz.setChecked(True)
        self.label_3 = QLabel(self.centralwidget)
        self.label_3.setObjectName(u"label_3")
        self.label_3.setGeometry(QRect(0, 280, 231, 16))
        self.tflist = QListWidget(self.centralwidget)
        self.tflist.setObjectName(u"tflist")
        self.tflist.setGeometry(QRect(0, 300, 321, 221))
        self.updatetf = QPushButton(self.centralwidget)
        self.updatetf.setObjectName(u"updatetf")
        self.updatetf.setGeometry(QRect(200, 530, 91, 28))
        eTeamDebugger.setCentralWidget(self.centralwidget)
        self.menuBar = QMenuBar(eTeamDebugger)
        self.menuBar.setObjectName(u"menuBar")
        self.menuBar.setGeometry(QRect(0, 0, 662, 22))
        self.menuFile = QMenu(self.menuBar)
        self.menuFile.setObjectName(u"menuFile")
        eTeamDebugger.setMenuBar(self.menuBar)

        self.menuBar.addAction(self.menuFile.menuAction())
        self.menuFile.addAction(self.importtopic)
        self.menuFile.addAction(self.exporttopic)
        self.menuFile.addSeparator()
        self.menuFile.addAction(self.importsnapshots)
        self.menuFile.addAction(self.exportsnapshots)

        self.retranslateUi(eTeamDebugger)

        QMetaObject.connectSlotsByName(eTeamDebugger)
    # setupUi

    def retranslateUi(self, eTeamDebugger):
        eTeamDebugger.setWindowTitle(QCoreApplication.translate("eTeamDebugger", u"eTeamDebugger", None))
        self.importtopic.setText(QCoreApplication.translate("eTeamDebugger", u"Import Topic List", None))
        self.exporttopic.setText(QCoreApplication.translate("eTeamDebugger", u"Export Topic List", None))
        self.importsnapshots.setText(QCoreApplication.translate("eTeamDebugger", u"Import Snapshots", None))
        self.exportsnapshots.setText(QCoreApplication.translate("eTeamDebugger", u"Export Snapshots", None))
        self.label.setText(QCoreApplication.translate("eTeamDebugger", u"List of Topics", None))
        self.label_2.setText(QCoreApplication.translate("eTeamDebugger", u"List of Snapshots", None))
        self.addtopic.setText(QCoreApplication.translate("eTeamDebugger", u"Add Topic", None))
        self.removetopic.setText(QCoreApplication.translate("eTeamDebugger", u"Remove Topic", None))
        self.newsnap.setText(QCoreApplication.translate("eTeamDebugger", u"Create", None))
        self.removesnap.setText(QCoreApplication.translate("eTeamDebugger", u"Remove", None))
        self.playsnap.setText(QCoreApplication.translate("eTeamDebugger", u"Publish", None))
        self.openrviz.setText(QCoreApplication.translate("eTeamDebugger", u"Open RViz", None))
        self.label_3.setText(QCoreApplication.translate("eTeamDebugger", u"List of TFs (selected is RViz default)", None))
        self.updatetf.setText(QCoreApplication.translate("eTeamDebugger", u"Update TFs", None))
        self.menuFile.setTitle(QCoreApplication.translate("eTeamDebugger", u"Topics", None))
    # retranslateUi

