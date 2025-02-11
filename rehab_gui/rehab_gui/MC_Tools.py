# -*- coding: utf-8 -*-
"""
Created on Mon Sep 17 13:15:24 2018

@author: marco
"""
from PyQt5 import QtGui
#from PyQt5.QtWidgets import QPushButton
import yaml

def enablePushButton(self, Enabled =1, State=1, FontSize = 9):
    self.State = State
    self.Enabled = Enabled
#    font = QtGui.QFont()
#    font.setPointSize(12)
    if Enabled == 1:
        self.setEnabled(True)
        self.setStyleSheet('Background: #cce4f7; font-size: %s' % FontSize + 'pt')
#        self.setFont(QtGui.setPointSize ( FontSize) )
        
    else:
        self.setEnabled(False)
        self.setStyleSheet('Background: #cce4f7; font-size: %s' % FontSize + 'pt')
#        self.setFont(QtGui.setPointSize ( FontSize) )

############################################################################
####                                                                   #####
####################### LOAD AND SAVE YAML FILES ###########################
####                                                                   #####
############################################################################

####  def LoadData(self, filename):
#
#        IN: filename  [string]  made of path and name of the file with extention  
#
#        OUT: Data [dictionary]      
##

def LoadData(self, filename):
    self.Data = yaml.load(open(filename))
    return self.Data

####  def SaveData(self, Data, filename):
#
#        IN: Data Data [dictionary] 
#            filename  [string]  made of path and name of the file with extention  
#
#        OUT:   
##

def SaveData(self, Data, filename):

    with open(filename, 'w') as outfile:
        yaml.dump(Data, outfile , default_flow_style=True)        

#class MC_PushButton(QPushButton):
#    
#    def enablePushButton(self, Enabled =1, State = 1):
#        self.Enabled = Enabled
#        self.State = State
#        if Enabled == 1:
#            self.setEnabled(True)
#            self.setStyleSheet("Background: #cce4f7")
#        else:
#            self.setEnabled(False)
        
        
#if __name__ == "__main__":
#    pass